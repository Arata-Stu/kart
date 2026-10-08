#pragma once
#include <chrono>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <kart_interfaces/msg/control_command.hpp>
#include <kart_interfaces/msg/operation_mode_request.hpp>
#include <kart_interfaces/msg/operation_mode_state.hpp>
#include "kart_system/control_core.hpp"
namespace kart_system {
using Control = kart_interfaces::msg::ControlCommand;
using Mode = kart_interfaces::msg::OperationModeState;
using Request = kart_interfaces::msg::OperationModeRequest;
inline double steady_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline double seconds(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
}
inline Command values(const Control & c) {return {c.steering,c.throttle,c.brake,c.reverse};}
inline void set_values(Control & msg, const Command & c) {
  msg.steering=c.steering; msg.throttle=c.throttle; msg.brake=c.brake; msg.reverse=c.reverse;
}
inline rcl_interfaces::msg::ParameterDescriptor fixed() {
  rcl_interfaces::msg::ParameterDescriptor d; d.read_only=true; return d;
}
inline double number(rclcpp::Node & n, const std::string & key, double value, double lo, double hi) {
  const auto result=n.declare_parameter<double>(key,value,fixed());
  if (!std::isfinite(result) || result < lo || result > hi) {
    throw std::invalid_argument(key + " outside allowed range");
  }
  return result;
}
struct Sample {
  Control msg;
  double received{NEVER};
  bool available(double steady, double ros, double timeout) const {
    return fresh(steady,received,timeout) && fresh_stamp(ros,seconds(msg.header.stamp),timeout) && valid(values(msg));
  }
};
inline void request(rclcpp::Node & node, const rclcpp::Publisher<Request>::SharedPtr & pub,
                    uint8_t mode, const std::string & source) {
  Request msg; msg.header.stamp=node.now(); msg.mode=mode; msg.source=source; pub->publish(msg);
}
}  // namespace kart_system
