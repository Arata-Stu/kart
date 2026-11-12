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
  bool r2_gate{false};
  static constexpr double ENTRY_NEUTRAL_S=0.3;
  static constexpr double ADJUST_STEP=0.001;
  static constexpr double R2_PRESS=0.10, R2_RELEASE=0.05;
  bool adjust(int direction, double limit, double ros, bool permitted) {
    if (!enabled() || !permitted || mode_!=STOP || !fresh_stamp(ros,mode_stamp_,0.3) ||
        (direction!=1 && direction!=-1) || !std::isfinite(limit) || limit<0.01 || limit>1 ||
        !valid_settings(limit)) {return false;}
    // Round only arithmetic noise; preserve deliberately configured non-grid values.
    throttle=std::clamp(std::round((throttle+direction*ADJUST_STEP)*1e9)/1e9,0.0,limit);
    return true;
  }
  bool enabled() const {return throttle >= 0;}
  bool valid_settings(double speed_scale) const {
    return std::isfinite(throttle) && ((!r2_gate && throttle == -1.0) ||
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
    if (mode!=AUTO) {
      reset_trigger();
      record_phase(healthy ? Phase::Inactive : Phase::Fault);
    }
    if (mode==AUTO && previous!=AUTO) {
      reset_trigger();
      // A gated run always requires a new release/press after entry. Direct
      // MANUAL handover must not turn a previously held R2 into fixed drive.
      if (authority_.active()) {
        entered_= previous==MANUAL && !r2_gate ? steady-ENTRY_NEUTRAL_S : steady;
        record_phase(Phase::EntryNeutral);
      } else {record_phase(Phase::Fault);}
    }
  }
  bool allowed(double ros, double steady, bool healthy) {
    if (mode_!=AUTO) {return false;}
    if ((std::isfinite(last_ros_) && ros<last_ros_) ||
        (std::isfinite(last_steady_) && (steady<last_steady_ || steady-last_steady_>0.3))) {trip();}
    last_ros_=ros; last_steady_=steady;
    if (!authority_.check(steady,0.3,healthy && fresh_stamp(ros,mode_stamp_,0.3))) {
      trip(); return false;
    }
    return true;
  }
  ManualThrottleResult command(Command raw, bool reverse_pressed,
                               double ros, double steady, bool healthy,
                               double raw_r2=0) {
    if (!enabled() || mode_!=AUTO) {record_phase(Phase::Inactive); return {raw,false};}
    if (!allowed(ros,steady,healthy) || !valid(raw) ||
        (r2_gate && (!std::isfinite(raw_r2) || raw_r2<0 || raw_r2>1))) {
      trip(); return {{},true};
    }
    if (raw.brake>0 || reverse_pressed) {
      trip();
      // Dedicated brake has priority; L2 cancels fixed propulsion without
      // requesting reverse. Neither release can automatically resume driving.
      return {{0,0,raw.brake,0},true};
    }
    if (r2_gate) {
      if (raw_r2>=R2_PRESS) {r2_pressed_=true;}
      else if (raw_r2<=R2_RELEASE) {r2_pressed_=false;}
    }
    if (steady-entered_<ENTRY_NEUTRAL_S) {
      trigger_ready_=false;
      record_phase(Phase::EntryNeutral);
      return {{},false};
    }
    if (r2_gate) {
      if (raw_r2<=R2_RELEASE) {trigger_ready_=true;}
      if (!trigger_ready_ || !r2_pressed_) {
        raw.throttle=raw.brake=raw.reverse=0;
        record_phase(Phase::Coast);
        return {raw,false};
      }
    }
    raw.throttle=throttle; raw.reverse=0;
    record_phase(Phase::Drive,raw.throttle);
    return {raw,false};
  }
  void trip() {authority_.trip(); reset_trigger(); record_phase(Phase::Fault);}
  bool active() const {return enabled() && mode_==AUTO && authority_.active();}
  bool r2_pressed() const {return r2_pressed_;}
  double effective_throttle() const {return effective_throttle_;}
  uint64_t phase_sequence() const {return phase_sequence_;}
  const char * phase() const {
    if (!enabled()) {return "disabled";}
    switch (phase_) {
      case Phase::Inactive: return "inactive";
      case Phase::EntryNeutral: return "entry_neutral";
      case Phase::Coast: return "coast";
      case Phase::Drive: return "drive";
      case Phase::Fault: return "fault";
    }
    return "fault";
  }
private:
  enum class Phase {Inactive,EntryNeutral,Coast,Drive,Fault};
  void reset_trigger() {r2_pressed_=false; trigger_ready_=false;}
  void record_phase(Phase phase,double throttle_value=0) {
    if (phase!=phase_) {phase_=phase; ++phase_sequence_;}
    effective_throttle_=throttle_value;
  }
  Authority authority_;
  Phase phase_{Phase::Inactive};
  bool r2_pressed_{false},trigger_ready_{false};
  double effective_throttle_{0};
  uint64_t phase_sequence_{0};
  uint8_t mode_{0};
  double mode_stamp_{NEVER},entered_{NEVER},last_ros_{NEVER},last_steady_{NEVER};
};
}  // namespace kart_system
