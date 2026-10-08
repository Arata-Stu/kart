#include "kart_vehicle/bridge_protocol.hpp"
#include "kart_vehicle/host_session.hpp"
#include "kart_vehicle/serial_port.hpp"
#include "kart_system/ros_helpers.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <kart_interfaces/msg/control_trim.hpp>
#include <memory>
#include <optional>

namespace kart_vehicle {
using namespace kart_system;
class BridgeNode : public rclcpp::Node {
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options) : Node("kart_bridge",options) {
    if (get_parameter("use_sim_time").as_bool()) {throw std::invalid_argument("physical bridge requires use_sim_time=false");}
    device_=declare_parameter<std::string>("device","",fixed());
    if(device_.empty()) {throw std::invalid_argument("device is required: use /dev/serial/by-id/... or an explicitly selected ttyACM path");}
    scale_=number(*this,"steering_scale",-1,-1,1);
    if(std::abs(scale_)<0.01) {throw std::invalid_argument("steering_scale must be nonzero");}
    offset_=number(*this,"steering_offset",0,-0.5,0.5);
    command_timeout_=number(*this,"command_timeout",0.15,0.02,0.2);
    status_timeout_=number(*this,"status_timeout",0.2,0.05,0.5);
    steering_trim_limit_=number(*this,"steering_trim_limit",0.25,0,0.5);
    throttle_trim_limit_=number(*this,"throttle_trim_limit",0.1,0,0.5);
    trim_.steering=number(*this,"initial_steering_trim",0,-steering_trim_limit_,steering_trim_limit_);
    trim_.throttle=number(*this,"initial_throttle_trim",0,-throttle_trim_limit_,throttle_trim_limit_);
    trim_pub_=create_publisher<kart_interfaces::msg::ControlTrim>("vehicle/trim/state",1);
    trim_sub_=create_subscription<kart_interfaces::msg::ControlTrim>("vehicle/trim/request",10,
      [this](kart_interfaces::msg::ControlTrim::ConstSharedPtr msg) {
        const auto stamp=seconds(msg->header.stamp);
        if ((gate_.mode()!=STOP && gate_.mode()!=MANUAL) ||
            !fresh_stamp(now().seconds(),stamp,0.1) || stamp<=last_trim_stamp_ ||
            session_.phase()==HostSession::Phase::waiting_input ||
            session_.phase()==HostSession::Phase::neutral_sent) {return;}
        const auto previous_trim=trim_;
        if (trim_.adjust(msg->steering,msg->throttle,steering_trim_limit_,throttle_trim_limit_)) {
          last_trim_stamp_=stamp;
          RCLCPP_INFO(get_logger(),
            "Trim updated: steering=%+.3f -> %+.3f, throttle=%+.3f -> %+.3f",
            previous_trim.steering,trim_.steering,previous_trim.throttle,trim_.throttle);
        }
      });
    request_pub_=create_publisher<Request>("operation_mode/request",10);
    diag_pub_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",1);
    rc_pub_=create_publisher<std_msgs::msg::Int32MultiArray>("vehicle/rc_channels",1);
    pwm_pub_=create_publisher<std_msgs::msg::Int32MultiArray>("vehicle/output_channels",1);
    vbec_pub_=create_publisher<std_msgs::msg::Float32>("vehicle/vbec",1);
    path_pub_=create_publisher<std_msgs::msg::UInt8>("vehicle/active_path",1);
    propo_pub_=create_publisher<Control>("propo/control_cmd",1);
    command_sub_=create_subscription<Control>("vehicle/control_cmd",1,[this](Control::ConstSharedPtr msg) {command_={*msg,steady_now()};});
    mode_sub_=create_subscription<Mode>("operation_mode/state",1,[this](Mode::ConstSharedPtr msg) {
      if (!valid_mode(msg->mode) || !fresh_stamp(now().seconds(),seconds(msg->header.stamp),0.3)) {trip("invalid_mode"); return;}
      const bool before=gate_.active();
      const bool permit=healthy() && status_->active_path==ActivePath::disabled;
      gate_.observe(msg->mode,steady_now(),permit);
      if (!gate_.active()) {
        session_.stop();
        if(host_mode(msg->mode)) {trip("arming_requires_stop_and_healthy_disarmed_board");}
      } else if (!before) {
        // Mux publishes only after mode entry. Do not accept buffered pre-entry data.
        command_.received=NEVER; session_.start(steady_now());
      }
    });
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this] {tick();});
  }
  ~BridgeNode() override {
    if(serial_) {try {CommandFrame frame; frame.sequence=sequence_++; frame.flags=COMMAND_VALID; serial_->write_line(encode_command(frame));} catch(...) {}}
  }
private:
  bool status_fresh() const {return status_ && fresh(steady_now(),status_at_,status_timeout_);}
  bool healthy() const {
    return serial_ && status_fresh() && status_->fault_bits==0 &&
      status_->selector==RcSelector::automatic && status_->active_path!=ActivePath::manual && status_->active_path!=ActivePath::failsafe;
  }
  void trip(const std::string & reason) {
    gate_.trip(); session_.stop(); error_=reason;
    if(steady_now()-last_request_>=0.1) {request(*this,request_pub_,STOP,"kart_bridge:"+reason); last_request_=steady_now();}
  }
  void tick() {
    const auto t=steady_now();
    try {
      if(get_parameter("use_sim_time").as_bool()) {throw std::runtime_error("use_sim_time_not_allowed");}
      if(!serial_ && t>=retry_at_) {
        retry_at_=t+1;
        serial_=std::make_unique<SerialPort>(device_); opened_at_=t; status_.reset();
        status_sequence_=StatusSequence{}; command_.received=NEVER;
        trip("usb_connected_requires_rearm");
        RCLCPP_INFO(get_logger(),"Opened %s; waiting for board status and explicit rearm",device_.c_str());
      }
      if(serial_) {
        for(const auto & line : serial_->read_lines()) {on_status(line);}
        if(t-opened_at_>status_timeout_ && !status_fresh()) {throw std::runtime_error("board_status_timeout");}
        if(gate_.active() && !gate_.check(t,0.3,healthy())) {trip("mode_or_board_unavailable");}
        const bool active=gate_.active();
        auto frame=session_.output(values(command_.msg),command_.available(t,now().seconds(),command_timeout_),t,scale_,offset_,trim_,gate_.mode()==MANUAL);
        if(active && session_.phase()==HostSession::Phase::disarmed) {trip("command_or_handshake_failed");}
        frame.sequence=sequence_++;
        serial_->write_line(encode_command(frame));
      }
    } catch(const std::exception & e) {
      serial_.reset(); status_.reset(); command_.received=NEVER; retry_at_=t+1;
      trip(e.what());
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Bridge disconnected: %s",e.what());
    }
    if(session_.esc().state()!=last_esc_state_) {
      RCLCPP_INFO(get_logger(),"ESC command model: %s (not measured motion)",session_.esc().name());
      last_esc_state_=session_.esc().state();
    }
    if(session_.esc().brake_rejected()) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,
        "Brake rejected: no forward/brake command history; sending neutral, not confirmed stopped");
    }
    if(t-last_trim_publish_>=0.1) {
      kart_interfaces::msg::ControlTrim msg; msg.header.stamp=now();
      msg.steering=trim_.steering; msg.throttle=trim_.throttle; trim_pub_->publish(msg); last_trim_publish_=t;
    }
    if(t-last_diag_>=1) {diagnostics(); last_diag_=t;}
  }
  void on_status(const std::string & line) {
    auto parsed=parse_status(line);
    if(!parsed) {++protocol_errors_; return;}
    if(!status_sequence_.accept(parsed->sequence)) {++protocol_errors_; return;}
    status_=*parsed; status_at_=steady_now();
    const bool active=gate_.active();
    session_.status(*parsed);
    if(active && (!healthy() || session_.phase()==HostSession::Phase::disarmed)) {trip("board_interlock");}
    if(parsed->active_path==ActivePath::manual && parsed->selector==RcSelector::propo && steady_now()-last_propo_>=0.1) {
      gate_.trip(); session_.stop(); request(*this,request_pub_,PROPO,"kart_bridge:physical_rc"); last_propo_=steady_now();
    }
    std_msgs::msg::Int32MultiArray rc; rc.data={parsed->rx1_us,parsed->rx2_us,parsed->rx3_us}; rc_pub_->publish(rc);
    std_msgs::msg::Int32MultiArray pwm; pwm.data={parsed->servo_output_us,parsed->esc_output_us}; pwm_pub_->publish(pwm);
    std_msgs::msg::Float32 vbec; vbec.data=parsed->vbec_mv/1000.0F; vbec_pub_->publish(vbec);
    std_msgs::msg::UInt8 path; path.data=static_cast<uint8_t>(parsed->active_path); path_pub_->publish(path);
    if(parsed->active_path==ActivePath::manual && parsed->selector==RcSelector::propo && !parsed->fault_bits) {
      const auto c=propo_pwm_to_command(parsed->rx1_us,parsed->rx2_us,PropoCalibration{});
      if(c) {Control msg; msg.header.stamp=now(); msg.header.frame_id="rc_receiver";
        msg.steering=c->steering; msg.throttle=c->throttle; msg.reverse=c->reverse; msg.brake=c->brake; propo_pub_->publish(msg);}
    }
  }
  void diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s; s.name="kart_bridge"; s.hardware_id="JPBB-01";
    s.level=!serial_ || !status_fresh() || status_->fault_bits ? s.ERROR : s.OK;
    s.message=!serial_ ? "USB serial disconnected" : !status_fresh() ? "board status timeout" :
      status_->fault_bits ? "board reported a fault" : "board communication healthy";
    auto add=[&](const std::string & key,const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv; kv.key=key; kv.value=value; s.values.push_back(kv);
    };
    add("steering_trim",std::to_string(trim_.steering)); add("throttle_trim",std::to_string(trim_.throttle));
    add("device",device_); add("host_active",gate_.active()?"true":"false");
    add("host_phase",std::to_string(static_cast<int>(session_.phase())));
    add("esc_command_state",session_.esc().name());
    add("esc_brake_rejected",session_.esc().brake_rejected()?"true":"false");
    add("protocol_errors",std::to_string(protocol_errors_)); add("last_interlock",error_);
    add("command_fresh",command_.available(steady_now(),now().seconds(),command_timeout_)?"true":"false");
    if(status_) {add("fault_bits",std::to_string(status_->fault_bits)); add("selector",std::to_string(static_cast<int>(status_->selector)));}
    array.status.push_back(s); diag_pub_->publish(array);
  }
  std::string device_,error_{"waiting_for_usb"}; double scale_,offset_,command_timeout_,status_timeout_;
  double retry_at_{0},opened_at_{0},status_at_{NEVER},last_request_{NEVER},last_propo_{NEVER},last_diag_{NEVER};
  ControlTrim trim_; double steering_trim_limit_,throttle_trim_limit_;
  double last_trim_stamp_{NEVER},last_trim_publish_{NEVER};
  uint32_t sequence_{0}; uint64_t protocol_errors_{0};
  Authority gate_; HostSession session_; StatusSequence status_sequence_; Sample command_;
  EscState::State last_esc_state_{EscState::State::unknown};
  std::unique_ptr<SerialPort> serial_; std::optional<StatusFrame> status_;
  rclcpp::Publisher<kart_interfaces::msg::ControlTrim>::SharedPtr trim_pub_;
  rclcpp::Subscription<kart_interfaces::msg::ControlTrim>::SharedPtr trim_sub_;
  rclcpp::Publisher<Request>::SharedPtr request_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rc_pub_,pwm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vbec_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr path_pub_;
  rclcpp::Publisher<Control>::SharedPtr propo_pub_;
  rclcpp::Subscription<Control>::SharedPtr command_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(kart_vehicle::BridgeNode)
