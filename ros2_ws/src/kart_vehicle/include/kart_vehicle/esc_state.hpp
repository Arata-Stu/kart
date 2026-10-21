#pragma once
#include "kart_vehicle/bridge_protocol.hpp"

namespace kart_vehicle {
// Issued-command model, not ESC feedback or measured vehicle motion.
// Neutral variants retain the preceding protocol state, regardless of elapsed time.
class EscState {
public:
  enum class State {unknown, neutral_initial, forward, neutral_after_forward,
                    brake, neutral_after_brake, reverse, neutral_after_reverse};
  void reset() {state_=State::unknown; brake_rejected_=false;}
  State state() const {return state_;}
  bool is_neutral() const {
    return state_==State::neutral_initial || state_==State::neutral_after_forward ||
      state_==State::neutral_after_brake || state_==State::neutral_after_reverse;
  }
  bool negative_is_brake() const {
    return state_==State::forward || state_==State::neutral_after_forward || state_==State::brake;
  }
  const char * next_negative_name() const {
    return state_==State::unknown ? "unknown" : negative_is_brake() ? "brake" : "reverse";
  }
  bool brake_rejected() const {return brake_rejected_;}
  const char * name() const {
    switch (state_) {
      case State::neutral_initial:return "neutral_initial";
      case State::neutral_after_forward:return "neutral_after_forward";
      case State::neutral_after_brake:return "neutral_after_brake";
      case State::neutral_after_reverse:return "neutral_after_reverse";
      case State::forward:return "forward";
      case State::brake:return "brake";
      case State::reverse:return "reverse";
      default:return "unknown";
    }
  }
  // HostSession validates exclusivity/range/freshness; caller handles timed holds.
  CommandFrame apply(CommandFrame frame) {
    brake_rejected_=false;
    if (!(frame.flags & COMMAND_VALID) || !(frame.flags & HOST_REQUEST)) {reset();return frame;}
    // Board integer milli/2 conversion: both 0 and 1 are physically neutral PWM.
    if(frame.throttle_milli<2)frame.throttle_milli=0;
    if(frame.reverse_milli<2)frame.reverse_milli=0;
    if(frame.brake_milli<2)frame.brake_milli=0;
    if(frame.throttle_milli==0 && frame.reverse_milli==0 && frame.brake_milli==0) {
      observe_neutral();return frame;
    }
    if(state_==State::unknown) {
      frame.throttle_milli=frame.reverse_milli=frame.brake_milli=0;return frame;
    }
    const bool can_brake=negative_is_brake();
    if(frame.brake_milli>0 && !can_brake) {
      // A brake request must not become reverse after brake release or reverse.
      frame.brake_milli=0;observe_neutral();brake_rejected_=true;
    } else if(frame.throttle_milli>0) {
      state_=State::forward;
    } else if(frame.brake_milli>0 || frame.reverse_milli>0) {
      if(can_brake) {
        frame.brake_milli+=frame.reverse_milli;frame.reverse_milli=0;state_=State::brake;
      } else {state_=State::reverse;}
    }
    return frame;
  }
private:
  void observe_neutral() {
    if(state_==State::forward)state_=State::neutral_after_forward;
    else if(state_==State::brake)state_=State::neutral_after_brake;
    else if(state_==State::reverse)state_=State::neutral_after_reverse;
    else if(state_==State::unknown)state_=State::neutral_initial;
    // Repeated neutral preserves its subtype indefinitely.
  }
  State state_{State::unknown};
  bool brake_rejected_{false};
};
} // namespace kart_vehicle
