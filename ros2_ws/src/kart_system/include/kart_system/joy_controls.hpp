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

// MANUAL circle-button signal generator. Time is steady seconds; caller owns
// freshness, mode, deadman and STOP gates. Reset requires a release before a pulse.
class JoyBrakePulse {
public:
  double throttle{0.05}, duration_s{0.02};
  bool valid_settings() const {
    return std::isfinite(throttle) && throttle>=0.002 && throttle<=1 &&
      std::isfinite(duration_s) && duration_s>=0.01 && duration_s<=0.5;
  }
  void reset() {released_=false; held_=false; pulsing_=false; started_=last_=NEVER;}
  const char * phase() const {return !held_ ? "idle" : pulsing_ ? "positive_pulse" : "brake";}
  Command command(Command raw, bool pressed, bool permitted, double steady) {
    if (!permitted) {reset(); return raw;}
    if (!valid_settings() || !valid(raw) || !std::isfinite(steady)) {reset(); return {};}
    if (std::isfinite(last_) && (steady<last_ || steady-last_>0.1)) {
      reset(); last_=steady;
      return pressed ? Command{raw.steering,0,1,0} : Command{};
    }
    last_=steady;
    if (!pressed) {released_=true; held_=false; pulsing_=false; return raw;}
    if (!held_) {
      held_=true;
      pulsing_=released_;
      started_=steady;
      released_=false;
    }
    if (steady-started_+1e-9>=duration_s) {pulsing_=false;}
    return pulsing_ ? Command{raw.steering,throttle,0,0} : Command{raw.steering,0,1,0};
  }
private:
  bool released_{false}, held_{false}, pulsing_{false};
  double started_{NEVER}, last_{NEVER};
};

struct ButtonActions {
  bool deadman{false}, brake{false}, stop{false}, manual{false}, automatic{false};
  bool manual_held{false}, auto_held{false}, stop_held{false};
  bool bag_start{false}, bag_stop{false};
  bool vgl_trigger{false};
  double steering_delta{0}, throttle_delta{0};
  int fixed_throttle_direction{0};
};

// Circle owns a separate hold-to-brake session. Never resumes ordinary driving.
// The adapter supplies fresh mode and board HOST-path feedback, and handles STOP.
class JoyBrakeHold {
public:
  struct Result {Command command{}; bool handled{false}, stop{false}, manual{false};};
  JoyBrakePulse pulse;
  bool active() const {return phase_!=Phase::idle;}
  void reset() {phase_=Phase::idle; released_=false; pulse.reset(); last_=NEVER;}
  Result update(bool pressed, bool healthy, uint8_t mode, bool host_ready,
                double steering, double now) {
    if (!healthy || !std::isfinite(now) ||
        (std::isfinite(last_) && (now<last_ || now-last_>0.1))) {
      const bool was_active=active(); reset(); return {{},pressed || was_active,was_active,false};
    }
    last_=now;
    if (!pressed) {
      const bool was_active=active(); reset(); released_=true; last_=now;
      return {{},was_active,was_active,false};
    }
    if (!active()) {
      if (!released_) {return {{},true,false,false};}
      released_=false;
      if (mode!=STOP && !host_mode(mode)) {return {{},true,true,false};}
      phase_=Phase::prepare; started_=now;
      pulse.reset(); pulse.command({},false,true,now);
      // An already active manual session needs no mode handover.
      if (mode==MANUAL && host_ready) {phase_=Phase::running;}
    }
    if (phase_!=Phase::running && now-started_>0.5) {reset(); return {{},true,true,false};}
    if (phase_==Phase::prepare) {
      // Let the mux receive neutral before requesting STOP -> MANUAL.
      if (now-started_+1e-9>=0.05) {
        phase_=Phase::waiting; requested_=now;
        return {{},true,false,true};
      }
      return {{},true,false,false};
    }
    if (phase_==Phase::waiting) {
      if (mode!=STOP && mode!=AUTO && mode!=MANUAL) {reset(); return {{},true,true,false};}
      if (mode!=MANUAL || !host_ready || now-requested_<0.05) {
        // Prime the pulse without spending its duration during handshake.
        pulse.command({},false,true,now);
        return {{},true,false,false};
      }
      phase_=Phase::running;
    }
    if (mode!=MANUAL || !host_ready) {reset(); return {{},true,true,false};}
    return {pulse.command({steering,0,1,0},true,true,now),true,false,false};
  }
private:
  enum class Phase {idle, prepare, waiting, running};
  Phase phase_{Phase::idle};
  bool released_{false};
  double started_{NEVER}, requested_{NEVER}, last_{NEVER};
};

// Mode buttons are hold-to-run. A fault/reset requires releasing both buttons;
// no repeated mode request may rearm a STOP while the operator keeps holding.
class JoyModeHold {
public:
  void reset() {selected_=STOP; locked_=true;}
  bool active() const {return host_mode(selected_);}
  uint8_t update(const ButtonActions & a) {
    if (a.stop || (a.manual_held && a.auto_held)) {
      reset(); return STOP;
    }
    if (!a.manual_held && !a.auto_held) {
      const bool released=host_mode(selected_);
      selected_=STOP; locked_=false;
      return released ? STOP : 0;
    }
    if (locked_) {return 0;}
    if (a.manual && a.manual_held) {selected_=MANUAL; return MANUAL;}
    if (a.automatic && a.auto_held) {selected_=AUTO; return AUTO;}
    if ((selected_==MANUAL && !a.manual_held) ||
        (selected_==AUTO && !a.auto_held)) {
      reset(); return STOP;
    }
    return 0;
  }
private:
  uint8_t selected_{STOP};
  bool locked_{true};
};

// Discrete inputs only. First sample after startup/reset establishes the held
// state; it must not synthesize a mode request or trim step on reconnection.
class JoyButtons {
public:
  int deadman_button{-1}, brake_button{1}, stop_button{0}, manual_button{2}, auto_button{3};
  int bag_start_button{4}, bag_stop_button{5};
  int vgl_trigger_button{8};
  int steering_left{15}, steering_right{16}, throttle_up{13}, throttle_down{14};
  double steering_step{0.001}, throttle_step{0.001};

  void reset() {primed_=false; previous_.clear();}
  std::optional<ButtonActions> read(const std::vector<int32_t> & buttons) {
    for (int index : {deadman_button,brake_button,stop_button,manual_button,
                     auto_button,bag_start_button,bag_stop_button,vgl_trigger_button,steering_left,steering_right,throttle_up,throttle_down}) {
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
    result.stop_held=pressed(stop_button);
    result.manual_held=pressed(manual_button); result.auto_held=pressed(auto_button);
    result.bag_stop=edge(bag_stop_button); result.bag_start=edge(bag_start_button) && !result.bag_stop;
    result.vgl_trigger=edge(vgl_trigger_button);
    result.steering_delta=(static_cast<int>(edge(steering_left))-static_cast<int>(edge(steering_right)))*steering_step;
    result.throttle_delta=(static_cast<int>(edge(throttle_up))-static_cast<int>(edge(throttle_down)))*throttle_step;
    if (!(pressed(throttle_up) && pressed(throttle_down))) {
      result.fixed_throttle_direction=static_cast<int>(edge(throttle_up))-static_cast<int>(edge(throttle_down));
    }
    previous_=buttons; primed_=true;
    return result;
  }
private:
  bool primed_{false};
  std::vector<int32_t> previous_;
};
}  // namespace kart_system
