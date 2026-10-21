#pragma once
#include "kart_vehicle/esc_state.hpp"
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>

namespace kart_vehicle {
// Open-loop protocol execution. Neither a pulse nor a dwell proves physical stop.
struct EscSequenceSettings {
  bool enabled{true};
  double reverse_brake_command{0.1};
  double reverse_brake_s{0.2};
  double reverse_neutral_s{0.01};
  double brake_entry_throttle{0.08};
  double brake_entry_s{0.05};
  double max_step_gap_s{0.1};
  void validate() const {
    for(double v : {reverse_brake_command,reverse_brake_s,reverse_neutral_s,brake_entry_throttle,brake_entry_s,max_step_gap_s}) {
      if(!std::isfinite(v) || v<=0) {throw std::invalid_argument("ESC sequence values must be finite and positive");}
    }
    if(reverse_brake_command<0.002 || reverse_brake_command>1 ||
       reverse_brake_s<0.001 || reverse_brake_s>2 ||
       brake_entry_throttle<0.002 || brake_entry_throttle>1 ||
       reverse_neutral_s<0.001 || reverse_neutral_s>2 ||
       brake_entry_s<0.001 || brake_entry_s>0.5 ||
       max_step_gap_s<0.001 || max_step_gap_s>0.2) {
      throw std::invalid_argument("ESC sequence settings out of range");
    }
  }
};
class EscSequencer {
public:
  enum class Phase {idle, reverse_braking, reverse_neutral, brake_entry, braking, rejected, failed};
  explicit EscSequencer(EscSequenceSettings settings={}) : settings_(settings) {settings_.validate();}
  void configure(EscSequenceSettings settings) {settings.validate();settings_=settings;reset();}
  void reset() {esc_.reset();phase_=Phase::idle;last_=since_=0;seen_=failed_=false;reason_.clear();}
  const EscState & esc() const {return esc_;}
  bool failed() const {return failed_;}
  const std::string & reason() const {return reason_;}
  const char * name() const {
    switch(phase_) {
      case Phase::reverse_braking:return "reverse_braking";
      case Phase::reverse_neutral:return "reverse_neutral";
      case Phase::brake_entry:return "brake_entry";
      case Phase::braking:return "braking";
      case Phase::rejected:return "rejected";
      case Phase::failed:return "failed";
      default:return "idle";
    }
  }
  // HostSession validates exclusivity/ranges and source freshness before every call.
  CommandFrame apply(CommandFrame frame,double now) {
    if(!(frame.flags & COMMAND_VALID) || !(frame.flags & HOST_REQUEST)) {reset();return esc_.apply(frame);}
    if(!settings_.enabled) {return esc_.apply(frame);}
    if(failed_) {return disabled(frame);}
    if(!std::isfinite(now) || (seen_ && (now<last_ || now-last_>settings_.max_step_gap_s))) {
      failed_=true;phase_=Phase::failed;reason_="clock_or_output_gap";esc_.reset();return disabled(frame);
    }
    seen_=true;last_=now;
    // Same quantization boundary as the board and EscState; tiny values are neutral.
    if(frame.throttle_milli<2)frame.throttle_milli=0;
    if(frame.reverse_milli<2)frame.reverse_milli=0;
    if(frame.brake_milli<2)frame.brake_milli=0;
    if(frame.throttle_milli>0) {phase_=Phase::idle;reason_.clear();return esc_.apply(frame);}
    if(frame.brake_milli>0) {
      if(phase_==Phase::brake_entry) {
        if(now-since_<settings_.brake_entry_s) {return pulse(frame);}
        phase_=Phase::braking;
        return esc_.apply(frame); // Direct positive -> negative, with no neutral sample.
      }
      const auto state=esc_.state();
      if(state==EscState::State::reverse) {
        phase_=Phase::brake_entry;since_=now;reason_.clear();return pulse(frame);
      }
      if(state==EscState::State::forward || state==EscState::State::brake) {
        phase_=Phase::braking;reason_.clear();return esc_.apply(frame);
      }
      phase_=Phase::rejected;reason_="brake_direction_unknown";
      return esc_.apply(frame); // Neutral/unknown must not invent a forward pulse.
    }
    if(frame.reverse_milli>0) {
      const auto state=esc_.state();
      if(phase_==Phase::reverse_braking) {
        if(now-since_<settings_.reverse_brake_s) {return reverse_brake(frame);}
        phase_=Phase::reverse_neutral;since_=now;return neutral(frame);
      }
      if(phase_==Phase::reverse_neutral) {
        if(now-since_<settings_.reverse_neutral_s) {return neutral(frame);}
        phase_=Phase::idle;reason_.clear();return esc_.apply(frame);
      }
      if(state==EscState::State::forward || state==EscState::State::brake) {
        phase_=Phase::reverse_braking;since_=now;reason_.clear();return reverse_brake(frame);
      }
      if(state!=EscState::State::reverse && state!=EscState::State::unknown) {
        phase_=Phase::reverse_neutral;since_=now;reason_.clear();return neutral(frame);
      }
      phase_=Phase::idle;reason_.clear();return esc_.apply(frame);
    }
    // Neutral (including steering-only) cancels every pending sequence immediately.
    phase_=Phase::idle;reason_.clear();return neutral(frame);
  }
private:
  CommandFrame reverse_brake(CommandFrame frame) {
    frame.throttle_milli=frame.reverse_milli=0;
    frame.brake_milli=static_cast<int>(std::lround(settings_.reverse_brake_command*1000));
    return esc_.apply(frame);
  }
  CommandFrame pulse(CommandFrame frame) {
    frame.brake_milli=frame.reverse_milli=0;
    frame.throttle_milli=static_cast<int>(std::lround(settings_.brake_entry_throttle*1000));
    return esc_.apply(frame);
  }
  CommandFrame neutral(CommandFrame frame) {
    frame.throttle_milli=frame.brake_milli=frame.reverse_milli=0;return esc_.apply(frame);
  }
  static CommandFrame disabled(CommandFrame frame) {
    frame.flags=COMMAND_VALID;frame.steering_milli=0;
    frame.throttle_milli=frame.brake_milli=frame.reverse_milli=0;return frame;
  }
  EscSequenceSettings settings_;
  EscState esc_;
  Phase phase_{Phase::idle};
  double last_{0},since_{0};
  bool seen_{false},failed_{false};
  std::string reason_;
};
} // namespace kart_vehicle
