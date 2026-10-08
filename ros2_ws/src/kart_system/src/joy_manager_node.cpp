#include "kart_system/ros_helpers.hpp"
#include "kart_system/joy_controls.hpp"
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
    pub_=create_publisher<Control>("teleop/control_cmd",rclcpp::QoS(1).reliable());
    request_pub_=create_publisher<Request>("operation_mode/request",10);
    ready_sub_=create_subscription<std_msgs::msg::Bool>("joy/ready",1,[this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      ready_=msg->data; ready_at_=steady_now(); if (!ready_) {lose_input();}
    });
    mode_sub_=create_subscription<Mode>("operation_mode/state",1,[this](Mode::ConstSharedPtr msg) {
      mode_=msg->mode; if (mode_==STOP) {blocked_=false;}
    });
    joy_sub_=create_subscription<sensor_msgs::msg::Joy>("joy",1,[this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {on_joy(*msg);});
    timer_=create_wall_timer(std::chrono::milliseconds(10),[this] {
      if (!ready_ || !fresh(steady_now(),ready_at_,0.1) || !fresh(steady_now(),joy_at_,0.1)) {lose_input();}
    });
  }
private:
  int index(const std::string & key,int value) {
    const int result=declare_parameter<int>(key,value,fixed());
    if (result < ((key=="auto_button" || key=="deadman_button" || key=="bag_start_button" || key=="bag_stop_button") ? -1 : 0) || result > 63) {throw std::invalid_argument(key+" invalid index");}
    return result;
  }
  void lose_input() {
    buttons_.reset();
    if (mode_==MANUAL) {
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
    // Recording is independent of driving mode, deadman and STOP. R1 wins on simultaneous edges.
    if (actions->bag_start || actions->bag_stop) {
      kart_interfaces::msg::BagRequest msg; msg.header=joy.header;
      msg.command=actions->bag_stop ? msg.STOP : msg.START;
      msg.label="joy"; bag_pub_->publish(msg);
      RCLCPP_INFO(get_logger(),"Bag request: %s",actions->bag_stop ? "STOP" : "START");
    }
    if (actions->stop) {blocked_=true; request(*this,request_pub_,STOP,"joy_manager:stop_button"); return;}
    if (!actions->deadman) {if (mode_==MANUAL) {lose_input();} return;}
    if (blocked_) {return;}
    if ((mode_==STOP || mode_==MANUAL) && (actions->steering_delta!=0 || actions->throttle_delta!=0)) {
      kart_interfaces::msg::ControlTrim trim; trim.header=joy.header;
      trim.steering=actions->steering_delta; trim.throttle=actions->throttle_delta; trim_pub_->publish(trim);
    }
    const auto c=teleop_.command(joy.axes,actions->brake);
    Control msg; msg.header=joy.header; set_values(msg,c); pub_->publish(msg);
    if (teleop_.can_arm(joy.axes,actions->brake) && mode_==STOP) {
      if (actions->manual) {request(*this,request_pub_,MANUAL,"joy_manager:manual_button");}
      else if (actions->automatic) {request(*this,request_pub_,AUTO,"joy_manager:auto_button");}
    }
  }
  TeleopInput teleop_;
  JoyButtons buttons_;
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
