#pragma once
#include <cmath>
#include <string>
#include <algorithm>
#include <initializer_list>
#include <stdexcept>

namespace kart_control {
struct FixedSettings {
  std::string fixed_axes{"both"};
  double teleop_timeout_s{0.15};
  double steering{0}, throttle{0}, brake{0}, max_throttle{1.0};
  double mode_timeout_s{0.3}, entry_neutral_s{0.3}, duration_s{0};
};
inline void validate_fixed(const FixedSettings &s) {
  for (double v : {s.steering, s.throttle, s.brake, s.max_throttle,
                   s.mode_timeout_s, s.entry_neutral_s, s.duration_s, s.teleop_timeout_s})
    if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite fixed command setting");
  if ((s.fixed_axes!="both" && s.fixed_axes!="throttle" && s.fixed_axes!="steering") ||
      s.teleop_timeout_s<=0 || std::abs(s.steering)>1 || s.throttle<0 || s.throttle>s.max_throttle ||
      s.max_throttle<0 || s.max_throttle>1 || s.brake<0 || s.brake>1 ||
      (s.throttle>0 && s.brake>0) || s.mode_timeout_s<=0 ||
      s.entry_neutral_s<0.1 || s.duration_s<0)
    throw std::invalid_argument("Invalid fixed command/range/timing");
}
struct FixedDecision {
  double steering{0}, throttle{0}, brake{0};
  const char *status{"inactive"};
  bool request_stop{false};
  double reverse{0};
};

// Real-time only. Each run requires observing STOP, then a new AUTO transition.
// A timeout/fault cannot resume merely because AUTO heartbeats return.
class FixedCommandGate {
public:
  explicit FixedCommandGate(FixedSettings settings) : settings_(settings) {
    validate_fixed(settings_);
  }
  bool configurable(double ros,double wall) const {
    return mode_==3 && received_>=0 && wall>=received_ &&
      wall-received_<=settings_.mode_timeout_s && ros>=stamp_ &&
      ros-stamp_<=settings_.mode_timeout_s;
  }
  void configure(const FixedSettings &s,double ros,double wall) {
    validate_fixed(s);
    if (!configurable(ros,wall)) trip("stop_required");
    settings_=s;
  }
  void teleop(double steering,double throttle,double brake,double reverse,
              double stamp,double ros,double wall) {
    manual_valid_=std::isfinite(steering) && std::isfinite(throttle) &&
      std::isfinite(brake) && std::isfinite(reverse) && std::isfinite(stamp) &&
      std::abs(steering)<=1 && throttle>=0 && throttle<=1 && brake>=0 && brake<=1 &&
      reverse>=0 && reverse<=1 && (int(throttle>0)+int(brake>0)+int(reverse>0)<=1) &&
      stamp>0 && stamp<=ros && ros-stamp<=settings_.teleop_timeout_s;
    manual_={steering,throttle,brake,"active",false,reverse};
    manual_stamp_=stamp; manual_received_=wall;
  }
  void observe(int mode, double stamp, double ros, double wall) {
    if (!std::isfinite(stamp) || stamp<=0 || stamp>ros ||
        ros-stamp>settings_.mode_timeout_s || mode<1 || mode>4 ||
        (last_stamp_>0 && stamp<last_stamp_)) {
      trip("invalid_mode"); return;
    }
    if (received_>=0 && wall-received_>settings_.mode_timeout_s)
      trip("mode_timeout");
    const int previous=mode_;
    mode_=mode; stamp_=stamp; received_=wall; last_stamp_=stamp;
    if (mode==3) {
      armed_=false; saw_stop_=true; latched_=false; reason_="inactive";
    } else if (mode==1 && previous!=1) {
      armed_=previous==3 && saw_stop_ && !latched_;
      saw_stop_=false; entered_=wall;
      if (!armed_) trip("stop_required");
    } else if (mode!=1) {
      armed_=false; saw_stop_=false;
    }
  }
  FixedDecision step(double ros, double wall, bool competing) {
    if (!std::isfinite(ros) || !std::isfinite(wall)) {
      trip("invalid_clock"); return stopped();
    }
    if (last_wall_>=0 && (wall<last_wall_ || wall-last_wall_>settings_.mode_timeout_s))
      trip("control_deadline");
    if (last_ros_>=0 && ros<last_ros_) trip("clock_jump");
    last_ros_=ros; last_wall_=wall;
    if (received_<0 || wall<received_ || wall-received_>settings_.mode_timeout_s ||
        stamp_<=0 || ros<stamp_ || ros-stamp_>settings_.mode_timeout_s) {
      trip("mode_timeout"); return stopped();
    }
    if (mode_!=1) return {0,0,0,"inactive",false};
    if (competing) trip("competing_publisher");
    if (latched_ || !armed_) return stopped();
    if (settings_.fixed_axes!="both" && (!manual_valid_ ||
        wall<manual_received_ || wall-manual_received_>settings_.teleop_timeout_s ||
        ros<manual_stamp_ || ros-manual_stamp_>settings_.teleop_timeout_s)) {
      trip("teleop_timeout"); return stopped();
    }
    const double elapsed=wall-entered_;
    if (elapsed<settings_.entry_neutral_s) return {0,0,0,"auto_entry_neutral",false};
    if (settings_.duration_s>0 && elapsed-settings_.entry_neutral_s>=settings_.duration_s) {
      trip("duration_complete"); return stopped();
    }
    if (settings_.fixed_axes=="throttle")
      return {manual_.steering,settings_.throttle,settings_.brake,"active",false};
    if (settings_.fixed_axes=="steering")
      return {settings_.steering,std::min(manual_.throttle,settings_.max_throttle),
              manual_.brake,"active",false,std::min(manual_.reverse,settings_.max_throttle)};
    return {settings_.steering,settings_.throttle,settings_.brake,"active",false};
  }
private:
  void trip(const char *reason) {latched_=true; armed_=false; saw_stop_=false; reason_=reason;}
  FixedDecision stopped() const {return {0,0,0,reason_,mode_==1};}
  FixedSettings settings_;
  FixedDecision manual_;
  bool manual_valid_{false};
  double manual_stamp_{0},manual_received_{-1};
  int mode_{0};
  bool armed_{false}, saw_stop_{false}, latched_{true};
  double stamp_{0}, last_stamp_{0}, received_{-1}, entered_{0}, last_wall_{-1}, last_ros_{-1};
  const char *reason_{"stop_required"};
};
} // namespace kart_control
