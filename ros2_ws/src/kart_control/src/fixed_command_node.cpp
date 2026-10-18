#include "kart_control/fixed_command.hpp"
#include "kart_interfaces/msg/control_command.hpp"
#include "kart_interfaces/msg/operation_mode_request.hpp"
#include "kart_interfaces/msg/operation_mode_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

namespace kart_control {
class FixedCommandNode : public rclcpp::Node {
public:
  explicit FixedCommandNode(const rclcpp::NodeOptions &options)
      : Node("fixed_command",options) {
    if (get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("Fixed-command vehicle trials require use_sim_time=false");
    FixedSettings s;
    s.fixed_axes=declare_parameter<std::string>("fixed_axes","both");
    s.teleop_timeout_s=parameter("teleop_timeout_s",0.15);
    s.steering=parameter("steering",0.0);
    s.throttle=parameter("throttle",0.0);
    s.brake=parameter("brake",0.0);
    s.max_throttle=parameter("max_throttle",0.2);
    s.mode_timeout_s=parameter("mode_timeout_s",0.3);
    s.entry_neutral_s=parameter("entry_neutral_s",0.3);
    s.duration_s=parameter("duration_s",0.0);
    double rate=parameter("control_rate_hz",50.0);
    if (!std::isfinite(rate) || rate<10 || rate>100)
      throw std::invalid_argument("control_rate_hz must be 10..100");
    settings_=s;
    gate_=std::make_unique<FixedCommandGate>(s);
    publisher_=create_publisher<kart_interfaces::msg::ControlCommand>("auto/control_cmd",1);
    stop_=create_publisher<kart_interfaces::msg::OperationModeRequest>("operation_mode/request",10);
    status_=create_publisher<std_msgs::msg::String>("control/fixed_status",1);
    config_=create_publisher<std_msgs::msg::String>("control/fixed_config",1);
    teleop_=create_subscription<kart_interfaces::msg::ControlCommand>("teleop/control_cmd",1,
      [this](kart_interfaces::msg::ControlCommand::ConstSharedPtr m) {
        gate_->teleop(m->steering,m->throttle,m->brake,m->reverse,
          rclcpp::Time(m->header.stamp).seconds(),now().seconds(),steady());
      });
    subscription_=create_subscription<kart_interfaces::msg::OperationModeState>(
        "operation_mode/state",1,[this](kart_interfaces::msg::OperationModeState::ConstSharedPtr m) {
          sync_settings();
          gate_->observe(m->mode,rclcpp::Time(m->header.stamp).seconds(),now().seconds(),steady());
        });
    // Default mutually-exclusive callback group serializes observe/tick even in an MT container.
    callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult result; result.successful=true;
      try {
        auto candidate=read_settings(); bool changed=false;
        for (const auto &p : params) {
          if (p.get_name()=="use_sim_time" && p.as_bool())
            throw std::invalid_argument("Fixed-command trials require the real clock");
          changed=assign(candidate,p) || changed;
        }
        validate_fixed(candidate);
        if (changed && !gate_->configurable(now().seconds(),steady()))
          throw std::invalid_argument("Change trial parameters only with a fresh STOP state");
      } catch (const std::exception &e) {result.successful=false;result.reason=e.what();}
      return result;
    });
    timer_=create_wall_timer(std::chrono::duration<double>(1/rate),[this]{tick();});
    RCLCPP_INFO(get_logger(),"Fixed AUTO trial: steer=%.3f throttle=%.3f brake=%.3f duration=%.3fs (0=unlimited); observe STOP before AUTO",
                s.steering,s.throttle,s.brake,s.duration_s);
  }
private:
  double parameter(const char *name,double value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only=std::string(name)!="steering" && std::string(name)!="throttle" &&
      std::string(name)!="brake" && std::string(name)!="duration_s";
    return declare_parameter<double>(name,value,descriptor);
  }
  static bool assign(FixedSettings &s,const rclcpp::Parameter &p) {
    const auto &n=p.get_name();
    if(n=="fixed_axes") s.fixed_axes=p.as_string();
    else if(n=="steering") s.steering=p.as_double();
    else if(n=="throttle") s.throttle=p.as_double();
    else if(n=="brake") s.brake=p.as_double();
    else if(n=="duration_s") s.duration_s=p.as_double();
    else return false;
    return true;
  }
  FixedSettings read_settings() const {
    auto s=settings_;
    for (const auto &name : {"fixed_axes","steering","throttle","brake","duration_s"})
      assign(s,get_parameter(name));
    return s;
  }
  void sync_settings() {
    const auto s=read_settings();
    if(s.fixed_axes==settings_.fixed_axes && s.steering==settings_.steering &&
       s.throttle==settings_.throttle && s.brake==settings_.brake && s.duration_s==settings_.duration_s) return;
    gate_->configure(s,now().seconds(),steady()); settings_=s; ++revision_; last_config_=-1;
  }
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void tick() {
    sync_settings();
    const auto time=now(); const double wall=steady();
    auto result=gate_->step(time.seconds(),wall,count_publishers(publisher_->get_topic_name())>1);
    kart_interfaces::msg::ControlCommand command;
    command.header.stamp=time; command.header.frame_id="base_link";
    command.steering=result.steering; command.throttle=result.throttle;
    command.brake=result.brake; command.reverse=result.reverse;
    publisher_->publish(command);
    if(wall-last_config_>=1.0) {
      std::ostringstream json; json << std::setprecision(17)
        << "{\"revision\":" << revision_ << ",\"fixed_axes\":\"" << settings_.fixed_axes
        << "\",\"steering\":" << settings_.steering << ",\"throttle\":" << settings_.throttle
        << ",\"brake\":" << settings_.brake << ",\"duration_s\":" << settings_.duration_s
        << ",\"max_throttle\":" << settings_.max_throttle << "}";
      std_msgs::msg::String config; config.data=json.str(); config_->publish(config);last_config_=wall;
    }
    if (wall-last_status_>=0.1) {
      std_msgs::msg::String status; status.data=result.status; status_->publish(status);
      if (result.request_stop) {
        kart_interfaces::msg::OperationModeRequest request;
        request.header.stamp=time; request.mode=request.STOP;
        request.source=std::string("fixed_command:")+result.status; stop_->publish(request);
      }
      last_status_=wall;
    }
  }
  FixedSettings settings_;
  unsigned long revision_{0};
  double last_config_{-1};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_;
  rclcpp::Subscription<kart_interfaces::msg::ControlCommand>::SharedPtr teleop_;
  double last_status_{-1};
  std::unique_ptr<FixedCommandGate> gate_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
  rclcpp::Subscription<kart_interfaces::msg::OperationModeState>::SharedPtr subscription_;
  rclcpp::Publisher<kart_interfaces::msg::ControlCommand>::SharedPtr publisher_;
  rclcpp::Publisher<kart_interfaces::msg::OperationModeRequest>::SharedPtr stop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
} // namespace kart_control
RCLCPP_COMPONENTS_REGISTER_NODE(kart_control::FixedCommandNode)
