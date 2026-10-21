#include "kart_system/ros_helpers.hpp"
#include "kart_system/joy_controls.hpp"
#include "kart_system/manual_throttle.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <kart_interfaces/msg/control_trim.hpp>
#include <kart_interfaces/msg/bag_request.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace kart_system {
class JoyManagerNode : public rclcpp::Node {
public:
  explicit JoyManagerNode(const rclcpp::NodeOptions & options) : Node("kart_joy_manager",options) {
    teleop_.steering_axis=index("steering_axis",0); teleop_.throttle_axis=index("throttle_axis",5); teleop_.reverse_axis=index("reverse_axis",4);
    buttons_.deadman_button=index("deadman_button",-1); buttons_.brake_button=index("brake_button",1);
    buttons_.stop_button=index("stop_button",0); buttons_.manual_button=index("manual_button",2); buttons_.auto_button=index("auto_button",3);
    buttons_.bag_start_button=index("bag_start_button",4); buttons_.bag_stop_button=index("bag_stop_button",5);
    bag_pub_=create_publisher<kart_interfaces::msg::BagRequest>("bag/request",10);
    buttons_.steering_left=index("steering_trim_left_button",15); buttons_.steering_right=index("steering_trim_right_button",16);
    buttons_.throttle_up=index("throttle_trim_up_button",13); buttons_.throttle_down=index("throttle_trim_down_button",14);
    buttons_.steering_step=number(*this,"steering_trim_step",0.001,0.001,0.05);
    buttons_.throttle_step=number(*this,"throttle_trim_step",0.001,0.001,0.05);
    trim_pub_=create_publisher<kart_interfaces::msg::ControlTrim>("vehicle/trim/request",10);
    teleop_.deadzone=number(*this,"deadzone",0.08,0,0.5);
    teleop_.steering_scale=number(*this,"steering_scale",1.0,0.01,1);
    teleop_.speed_scale=number(*this,"speed_scale",0.3,0.01,1);
    brake_pulse_.throttle=number(*this,"brake_pulse_throttle",0.08,0.002,1.0);
    brake_pulse_.duration_s=number(*this,"brake_pulse_s",0.01,0.01,0.5);
    manual_fixed_.throttle=number(*this,"auto_fixed_throttle",-1.0,-1.0,1.0);
    if (!manual_fixed_.valid_settings(teleop_.speed_scale)) {
      throw std::invalid_argument("auto_fixed_throttle must be -1 (disabled) or 0..speed_scale");
    }
    fixed_config_pub_=create_publisher<std_msgs::msg::String>("teleop/fixed_config",rclcpp::QoS(1).reliable());
    RCLCPP_INFO(get_logger(),"AUTO fixed throttle: %s; value=%.6f; limit(speed_scale)=%.6f",
      manual_fixed_.enabled() ? "enabled" : "disabled",manual_fixed_.throttle,teleop_.speed_scale);
    pub_=create_publisher<Control>("teleop/control_cmd",rclcpp::QoS(1).reliable());
    request_pub_=create_publisher<Request>("operation_mode/request",10);
    ready_sub_=create_subscription<std_msgs::msg::Bool>("joy/ready",1,[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      ready_=msg->data; ready_at_=steady_now(); if (!ready_) {lose_input();}
    });
    mode_sub_=create_subscription<Mode>("operation_mode/state",1,[this](Mode::ConstSharedPtr msg) {
      if (mode_!=msg->mode) {reset_brake_pulse("mode_changed");}
      mode_=msg->mode; mode_at_=steady_now(); mode_stamp_=seconds(msg->header.stamp);
      manual_fixed_.observe(mode_,seconds(msg->header.stamp),now().seconds(),steady_now(),
        input_healthy() && !blocked_,neutral_input_);
      if (mode_==STOP) {blocked_=false;}
    });
    joy_sub_=create_subscription<sensor_msgs::msg::Joy>("joy",1,[this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {on_joy(*msg);});
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this] {
      if (!input_healthy()) {lose_input();}
      else if (manual_fixed_.enabled() && mode_==AUTO &&
               !manual_fixed_.allowed(now().seconds(),steady_now(),!blocked_)) {
        stop_fixed({},"joy_manager:fixed_throttle_mode_unavailable");
      }
      if (!manual_brake_permitted()) {reset_brake_pulse("mode_or_input_unavailable");}
      if (steady_now()-last_config_>=0.1) {publish_fixed_config(); last_config_=steady_now();}
    });
  }
private:
  int index(const std::string & key,int value) {
    const int result=declare_parameter<int>(key,value,fixed());
    if (result < ((key=="auto_button" || key=="deadman_button" || key=="bag_start_button" || key=="bag_stop_button") ? -1 : 0) || result > 63) {throw std::invalid_argument(key+" invalid index");}
    return result;
  }
  bool input_healthy() const {
    return ready_ && fresh(steady_now(),ready_at_,0.1) && fresh(steady_now(),joy_at_,0.1);
  }
  bool manual_brake_permitted() const {
    return mode_==MANUAL && !blocked_ && input_healthy() &&
      fresh(steady_now(),mode_at_,0.3) && fresh_stamp(now().seconds(),mode_stamp_,0.3);
  }
  void log_brake_signal(double steady, const char * reason) {
    const std::string phase=brake_pulse_.phase();
    if (phase==brake_log_phase_) {return;}
    const double elapsed=std::isfinite(brake_log_at_) ? std::max(0.0,steady-brake_log_at_) : 0.0;
    if (phase=="positive_pulse") {
      RCLCPP_INFO(get_logger(),
        "Joy brake signal: positive_pulse throttle=+%.3f requested_s=%.3f -> brake=1.000 while circle held (pre-trim command)",
        brake_pulse_.throttle,brake_pulse_.duration_s);
    } else if (phase=="brake" && brake_log_phase_=="positive_pulse") {
      RCLCPP_INFO(get_logger(),
        "Joy brake signal: brake=1.000; positive_pulse requested_s=%.3f elapsed_s=%.6f; hold until circle release (command timing, not ESC feedback)",
        brake_pulse_.duration_s,elapsed);
    } else if (phase=="brake") {
      RCLCPP_INFO(get_logger(),
        "Joy brake signal: brake=1.000; positive pulse skipped (release circle before a new pulse)");
    } else {
      RCLCPP_INFO(get_logger(),
        "Joy brake signal: idle previous=%s phase_elapsed_s=%.6f reason=%s (command sequence ended)",
        brake_log_phase_.c_str(),elapsed,reason);
    }
    brake_log_phase_=phase; brake_log_at_=steady;
  }
  void reset_brake_pulse(const char * reason) {
    brake_pulse_.reset();
    log_brake_signal(steady_now(),reason);
  }
  void publish_fixed_config() {
    std_msgs::msg::String msg;
    msg.data="{\"schema\":\"kart.auto_fixed_throttle.v1\",\"enabled\":"+
      std::string(manual_fixed_.enabled() ? "true" : "false")+
      ",\"throttle\":"+std::to_string(manual_fixed_.throttle)+
      ",\"entry_neutral_s\":"+std::to_string(ManualFixedThrottle::ENTRY_NEUTRAL_S)+
      ",\"speed_scale\":"+std::to_string(teleop_.speed_scale)+
      ",\"mode\":"+std::to_string(mode_)+
      ",\"armed\":"+std::string(manual_fixed_.active() && !blocked_ ? "true" : "false")+"}";
    fixed_config_pub_->publish(msg);
  }
  void stop_fixed(const Command & command,const std::string & reason) {
    manual_fixed_.trip(); blocked_=true;
    Control msg; msg.header.stamp=now(); set_values(msg,command); pub_->publish(msg);
    if (steady_now()-last_stop_>=0.1) {request(*this,request_pub_,STOP,reason); last_stop_=steady_now();}
  }
  void lose_input() {
    reset_brake_pulse("input_unavailable"); buttons_.reset(); neutral_input_=false; manual_fixed_.trip();
    if (mode_==MANUAL || (mode_==AUTO && manual_fixed_.enabled())) {
      if (manual_fixed_.enabled() && mode_==AUTO) {stop_fixed({},"joy_manager:fixed_throttle_input_unavailable"); return;}
      blocked_=true;
      if (steady_now()-last_stop_>=0.1) {request(*this,request_pub_,STOP,"joy_manager:input_unavailable"); last_stop_=steady_now();}
    }
  }
  void on_joy(const sensor_msgs::msg::Joy & joy) {
    joy_at_=steady_now();
    if (!ready_ || !fresh(steady_now(),ready_at_,0.1) || !fresh_stamp(now().seconds(),seconds(joy.header.stamp),0.1)) {lose_input(); return;}
    if (!teleop_.valid_axes(joy.axes)) {lose_input(); return;}
    const auto actions=buttons_.read(joy.buttons);
    if (!actions) {lose_input(); return;}
    neutral_input_=teleop_.can_arm(joy.axes,actions->brake);
    // Recording is independent of driving mode, deadman and STOP. R1 wins on simultaneous edges.
    if (actions->bag_start || actions->bag_stop) {
      kart_interfaces::msg::BagRequest msg; msg.header=joy.header;
      msg.command=actions->bag_stop ? msg.STOP : msg.START;
      msg.label="joy"; bag_pub_->publish(msg);
      RCLCPP_INFO(get_logger(),"Bag request: %s",actions->bag_stop ? "STOP" : "START");
    }
    if (actions->stop) {
      reset_brake_pulse("stop_button");
      if (manual_fixed_.enabled() && mode_==AUTO) {stop_fixed({},"joy_manager:stop_button");}
      else {blocked_=true; request(*this,request_pub_,STOP,"joy_manager:stop_button");}
      return;
    }
    if (!actions->deadman) {if (host_mode(mode_)) {lose_input();} return;}
    if (blocked_) {return;}
    if ((mode_==STOP || mode_==MANUAL) && (actions->steering_delta!=0 || actions->throttle_delta!=0)) {
      kart_interfaces::msg::ControlTrim trim; trim.header=joy.header;
      trim.steering=actions->steering_delta;
      trim.throttle=actions->throttle_delta;
      if (trim.steering!=0 || trim.throttle!=0) {trim_pub_->publish(trim);}
    }
    if (actions->manual && mode_==AUTO) {
      reset_brake_pulse("manual_takeover");
      request(*this,request_pub_,MANUAL,"joy_manager:manual_takeover");
      Control msg; msg.header=joy.header; set_values(msg,teleop_.command(joy.axes,actions->brake)); pub_->publish(msg); return;
    }
    const auto raw=teleop_.command(joy.axes,actions->brake);
    const double brake_now=steady_now();
    const auto manual=brake_pulse_.command(raw,actions->brake,manual_brake_permitted(),brake_now);
    log_brake_signal(brake_now,"circle_release_or_control_gate");
    const auto result=manual_fixed_.command(manual,joy.axes[teleop_.reverse_axis]>0.01,
      now().seconds(),steady_now(),input_healthy() && !blocked_);
    if (result.stop) {stop_fixed(result.command,"joy_manager:fixed_throttle_cancelled"); return;}
    Control msg; msg.header=joy.header; set_values(msg,result.command); pub_->publish(msg);
    if ((mode_==STOP && teleop_.can_arm(joy.axes,actions->brake)) || host_mode(mode_)) {
      if (actions->manual) {request(*this,request_pub_,MANUAL,"joy_manager:manual_button");}
      else if (actions->automatic) {request(*this,request_pub_,AUTO,"joy_manager:auto_button");}
    }
  }
  std::string brake_log_phase_{"idle"};
  double brake_log_at_{NEVER};
  JoyBrakePulse brake_pulse_;
  double mode_at_{NEVER}, mode_stamp_{NEVER};
  TeleopInput teleop_;
  JoyButtons buttons_;
  ManualFixedThrottle manual_fixed_;
  bool neutral_input_{false};
  double last_config_{NEVER};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fixed_config_pub_;
  double ready_at_{NEVER},joy_at_{NEVER},last_stop_{NEVER};
  bool ready_{false},blocked_{false}; uint8_t mode_{STOP};
  rclcpp::Publisher<kart_interfaces::msg::BagRequest>::SharedPtr bag_pub_;
  rclcpp::Publisher<Control>::SharedPtr pub_;
  rclcpp::Publisher<kart_interfaces::msg::ControlTrim>::SharedPtr trim_pub_;
  rclcpp::Publisher<Request>::SharedPtr request_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace kart_system
RCLCPP_COMPONENTS_REGISTER_NODE(kart_system::JoyManagerNode)
