#pragma once
#include "kart_system/control_core.hpp"

namespace kart_system {
struct ManualThrottleResult {
  Command command;
  bool stop{false};
};

// Explicit opt-in for AUTO data collection. Authority uses the actual STOP ->
// AUTO transition; a node restart or input recovery must not start propulsion.
class ManualFixedThrottle {
public:
  double throttle{-1.0};
  static constexpr double ENTRY_NEUTRAL_S=0.3;
  bool enabled() const {return throttle >= 0;}
  bool valid_settings(double speed_scale) const {
    return std::isfinite(throttle) && (throttle == -1.0 ||
      (throttle >= 0 && throttle <= speed_scale));
  }
  void observe(uint8_t mode, double stamp, double ros, double steady,
               bool healthy, bool neutral_input) {
    const auto previous=mode_;
    const auto previous_stamp=mode_stamp_;
    mode_=mode; mode_stamp_=stamp;
    if (!valid_mode(mode) || !fresh_stamp(ros,stamp,0.3) ||
        (std::isfinite(previous_stamp) && stamp<previous_stamp)) {trip(); return;}
    if (mode==STOP || (mode==AUTO && previous!=AUTO)) {last_ros_=last_steady_=NEVER;}
    authority_.observe(mode,steady,healthy && (neutral_input || host_mode(previous)));
    if (mode==AUTO && previous!=AUTO && authority_.active()) {entered_= previous==MANUAL ? steady-ENTRY_NEUTRAL_S : steady;}
  }
  bool allowed(double ros, double steady, bool healthy) {
    if (mode_!=AUTO) {return false;}
    if ((std::isfinite(last_ros_) && ros<last_ros_) ||
        (std::isfinite(last_steady_) && (steady<last_steady_ || steady-last_steady_>0.3))) {trip();}
    last_ros_=ros; last_steady_=steady;
    return authority_.check(steady,0.3,healthy && fresh_stamp(ros,mode_stamp_,0.3));
  }
  ManualThrottleResult command(Command raw, bool reverse_pressed,
                               double ros, double steady, bool healthy) {
    if (!enabled() || mode_!=AUTO) {return {raw,false};}
    if (!allowed(ros,steady,healthy) || !valid(raw)) {trip(); return {{},true};}
    if (raw.brake>0 || reverse_pressed) {
      trip();
      // Dedicated brake has priority; L2 cancels fixed propulsion without
      // requesting reverse. Neither release can automatically resume driving.
      return {{0,0,raw.brake,0},true};
    }
    if (steady-entered_<ENTRY_NEUTRAL_S) {return {{},false};}
    raw.throttle=throttle; raw.reverse=0;
    return {raw,false};
  }
  void trip() {authority_.trip();}
  bool active() const {return enabled() && mode_==AUTO && authority_.active();}
private:
  Authority authority_;
  uint8_t mode_{0};
  double mode_stamp_{NEVER},entered_{NEVER},last_ros_{NEVER},last_steady_{NEVER};
};
}  // namespace kart_system
