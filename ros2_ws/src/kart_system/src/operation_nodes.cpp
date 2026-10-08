#include "kart_system/ros_helpers.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace kart_system {
class ModeManagerNode : public rclcpp::Node {
public:
  explicit ModeManagerNode(const rclcpp::NodeOptions & options) : Node("operation_mode_manager",options) {
    pub_=create_publisher<Mode>("operation_mode/state",rclcpp::QoS(1).reliable());
    sub_=create_subscription<Request>("operation_mode/request",10,[this](Request::ConstSharedPtr msg) {
      // STOP is fail-closed and may be sent without a timestamp from the CLI.
      if (!valid_mode(msg->mode) || (msg->mode != STOP &&
          !fresh_stamp(now().seconds(),seconds(msg->header.stamp),0.5))) {return;}
      // Host entry must start at STOP; repeated host requests never constitute rearming.
      if (host_mode(msg->mode) && mode_ != STOP && mode_ != msg->mode) {return;}
      mode_=msg->mode; source_=msg->source; publish();
    });
    timer_=create_wall_timer(std::chrono::milliseconds(100),[this] {publish();});
  }
private:
  void publish() {Mode msg; msg.header.stamp=now(); msg.mode=mode_; msg.source=source_; pub_->publish(msg);}
  uint8_t mode_{STOP}; std::string source_{"startup"};
  rclcpp::Publisher<Mode>::SharedPtr pub_;
  rclcpp::Subscription<Request>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class CommandMuxNode : public rclcpp::Node {
public:
  explicit CommandMuxNode(const rclcpp::NodeOptions & options) : Node("command_mux",options) {
    timeout_=number(*this,"command_timeout",0.15,0.02,0.2);
    pub_=create_publisher<Control>("vehicle/control_cmd",rclcpp::QoS(1).reliable());
    request_pub_=create_publisher<Request>("operation_mode/request",10);
    auto_sub_=create_subscription<Control>("auto/control_cmd",rclcpp::QoS(1).best_effort(),
      [this](Control::ConstSharedPtr msg) {automatic_={*msg,steady_now()};});
    manual_sub_=create_subscription<Control>("teleop/control_cmd",rclcpp::QoS(1).best_effort(),
      [this](Control::ConstSharedPtr msg) {manual_={*msg,steady_now()};});
    mode_sub_=create_subscription<Mode>("operation_mode/state",1,[this](Mode::ConstSharedPtr msg) {
      if (!valid_mode(msg->mode) || !fresh_stamp(now().seconds(),seconds(msg->header.stamp),0.3)) {trip("invalid_mode"); return;}
      const auto & input = msg->mode == AUTO ? automatic_ : manual_;
      gate_.observe(msg->mode,steady_now(),input.available(steady_now(),now().seconds(),timeout_) && neutral(values(input.msg)));
      if (host_mode(msg->mode) && !gate_.active()) {trip("arming_requires_stop_and_neutral_input");}
    });
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this] {tick();});
  }
private:
  void trip(const std::string & why) {
    gate_.trip();
    if (steady_now()-last_stop_ >= 0.1) {request(*this,request_pub_,STOP,"command_mux:"+why); last_stop_=steady_now();}
  }
  void tick() {
    if (!host_mode(gate_.mode())) {return;}
    const auto & input=gate_.mode()==AUTO ? automatic_ : manual_;
    if (!gate_.check(steady_now(),0.3,input.available(steady_now(),now().seconds(),timeout_))) {trip("input_or_mode_timeout"); return;}
    // Preserve source time; never rejuvenate an upstream command.
    pub_->publish(input.msg);
  }
  double timeout_{0.15}, last_stop_{NEVER}; Authority gate_; Sample automatic_,manual_;
  rclcpp::Publisher<Control>::SharedPtr pub_;
  rclcpp::Publisher<Request>::SharedPtr request_pub_;
  rclcpp::Subscription<Control>::SharedPtr auto_sub_,manual_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace kart_system
RCLCPP_COMPONENTS_REGISTER_NODE(kart_system::ModeManagerNode)
RCLCPP_COMPONENTS_REGISTER_NODE(kart_system::CommandMuxNode)
