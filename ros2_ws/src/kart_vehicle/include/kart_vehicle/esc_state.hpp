#pragma once
#include "kart_vehicle/bridge_protocol.hpp"

namespace kart_vehicle {
// A model of issued ESC commands, NOT measured motion or confirmed ESC state.
// Apply after milli-unit quantization, accounting for firmware PWM resolution.
// Caller validates range/exclusivity before calling (HostSession does so).
class EscState {
public:
  enum class State {unknown, neutral, forward, brake, reverse};
  void reset() {state_=State::unknown; brake_rejected_=false;}
  State state() const {return state_;}
  bool brake_rejected() const {return brake_rejected_;}
  const char * name() const {
    switch (state_) {
      case State::neutral: return "neutral";
      case State::forward: return "forward";
      case State::brake: return "brake";
      case State::reverse: return "reverse";
      default: return "unknown";
    }
  }
  CommandFrame apply(CommandFrame frame) {
    brake_rejected_=false;
    if (!(frame.flags & COMMAND_VALID) || !(frame.flags & HOST_REQUEST)) {
      reset(); return frame;
    }
    // Firmware uses integer milli/2 to offset neutral PWM in microseconds.
    // Both 0 and 1 therefore produce neutral, never a directional transition.
    if (frame.throttle_milli<2) {frame.throttle_milli=0;}
    if (frame.reverse_milli<2) {frame.reverse_milli=0;}
    if (frame.brake_milli<2) {frame.brake_milli=0;}
    const bool zero=frame.throttle_milli==0 && frame.reverse_milli==0 && frame.brake_milli==0;
    if (zero) {state_=State::neutral; return frame;}
    // Only a neutral HOST handshake may establish the initial command model.
    if (state_==State::unknown) {
      frame.throttle_milli=frame.reverse_milli=frame.brake_milli=0;
      return frame;
    }
    const bool can_brake=state_==State::forward || state_==State::brake;
    if (frame.brake_milli>0 && !can_brake) {
      // Brake and reverse have identical PWM. Never interpret an explicit
      // brake request from neutral/reverse as permission to drive backwards.
      frame.brake_milli=0;
      state_=State::neutral; brake_rejected_=true;
    } else if (frame.throttle_milli>0) {
      // Reverse -> forward has no separate brake stage on this ESC model.
      state_=State::forward;
    } else if (frame.brake_milli>0 || frame.reverse_milli>0) {
      if (can_brake) {
        frame.brake_milli+=frame.reverse_milli;
        frame.reverse_milli=0;
        state_=State::brake;
      } else {
        state_=State::reverse;
      }
    }
    return frame;
  }
private:
  State state_{State::unknown};
  bool brake_rejected_{false};
};
}  // namespace kart_vehicle
