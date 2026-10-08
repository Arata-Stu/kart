#pragma once
#include "kart_system/control_core.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace kart_system {
// Continuous inputs only. No ROS subscriptions or mode/connection state.
class TeleopInput {
public:
  int steering_axis{0}, throttle_axis{5}, reverse_axis{4};
  double deadzone{0.08}, steering_scale{1.0}, speed_scale{0.3};

  bool valid_axes(const std::vector<float> & axes) const {
    for (int index : {steering_axis, throttle_axis, reverse_axis}) {
      if (index<0 || static_cast<std::size_t>(index)>=axes.size() ||
          !std::isfinite(axes[index]) || std::abs(axes[index])>1) {return false;}
    }
    return axes[throttle_axis]>=0 && axes[reverse_axis]>=0;
  }
  // Call after valid_axes(). Keep the normalized command independent of trims.
  Command command(const std::vector<float> & axes, bool brake) const {
    return teleop(axes[steering_axis],axes[throttle_axis],axes[reverse_axis],
                  brake,deadzone,steering_scale,speed_scale);
  }
  bool can_arm(const std::vector<float> & axes, bool brake) const {
    return valid_axes(axes) && neutral(command(axes,brake)) &&
      axes[throttle_axis]<=0.01 && axes[reverse_axis]<=0.01;
  }
};

struct ButtonActions {
  bool deadman{false}, brake{false}, stop{false}, manual{false}, automatic{false};
  bool bag_start{false}, bag_stop{false};
  double steering_delta{0}, throttle_delta{0};
};

// Discrete inputs only. First sample after startup/reset establishes the held
// state; it must not synthesize a mode request or trim step on reconnection.
class JoyButtons {
public:
  int deadman_button{-1}, brake_button{1}, stop_button{0}, manual_button{2}, auto_button{3};
  int bag_start_button{4}, bag_stop_button{5};
  int steering_left{15}, steering_right{16}, throttle_up{13}, throttle_down{14};
  double steering_step{0.001}, throttle_step{0.001};

  void reset() {primed_=false; previous_.clear();}
  std::optional<ButtonActions> read(const std::vector<int32_t> & buttons) {
    for (int index : {deadman_button,brake_button,stop_button,manual_button,
                     auto_button,bag_start_button,bag_stop_button,steering_left,steering_right,throttle_up,throttle_down}) {
      if (index < -1 || (index>=0 && static_cast<std::size_t>(index)>=buttons.size())) {
        reset(); return std::nullopt;
      }
    }
    auto pressed=[&](int index) {return index>=0 && buttons[index]!=0;};
    auto edge=[&](int index) {
      return primed_ && pressed(index) && static_cast<std::size_t>(index)<previous_.size() && !previous_[index];
    };
    ButtonActions result;
    result.deadman=deadman_button==-1 || pressed(deadman_button); result.brake=pressed(brake_button);
    result.stop=edge(stop_button); result.manual=edge(manual_button); result.automatic=edge(auto_button);
    result.bag_stop=edge(bag_stop_button); result.bag_start=edge(bag_start_button) && !result.bag_stop;
    result.steering_delta=(static_cast<int>(edge(steering_left))-static_cast<int>(edge(steering_right)))*steering_step;
    result.throttle_delta=(static_cast<int>(edge(throttle_up))-static_cast<int>(edge(throttle_down)))*throttle_step;
    previous_=buttons; primed_=true;
    return result;
  }
private:
  bool primed_{false};
  std::vector<int32_t> previous_;
};
}  // namespace kart_system
