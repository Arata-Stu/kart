#pragma once
#include "kart_system/control_core.hpp"
#include "kart_vehicle/bridge_protocol.hpp"
#include "kart_vehicle/esc_state.hpp"
namespace kart_vehicle {
// Status sequence is a BOARD counter, not a host command acknowledgement.
class StatusSequence {
public:
  bool accept(uint32_t next) {
    const uint32_t delta=next-last_;
    if (seen_ && (delta==0 || delta>=0x80000000U)) {return false;}
    seen_=true; last_=next; return true;
  }
private:
  bool seen_{false}; uint32_t last_{0};
};
class HostSession {
public:
  enum class Phase {disarmed, waiting_input, neutral_sent, armed};
  void start(double now) {phase_=Phase::waiting_input; started_=now; esc_.reset();}
  void stop() {phase_=Phase::disarmed; esc_.reset();}
  const EscState & esc() const {return esc_;}
  // Called only for a new, CRC-valid, fresh board frame received after a write.
  void status(const StatusFrame & s) {
    if (s.fault_bits || s.selector!=RcSelector::automatic || s.active_path==ActivePath::failsafe || s.active_path==ActivePath::manual) {stop();}
    else if (phase_==Phase::neutral_sent && s.active_path==ActivePath::automatic) {phase_=Phase::armed;}
    else if (phase_==Phase::armed && s.active_path!=ActivePath::automatic) {stop();}
  }
  CommandFrame output(const kart_system::Command & c, bool fresh, double now,
                      double scale, double offset,
                      const kart_system::ControlTrim & trim = {}, bool manual = false) {
    CommandFrame frame; frame.flags=COMMAND_VALID;
    if (phase_==Phase::disarmed) {return frame;}
    if (phase_!=Phase::armed && now-started_>0.5) {stop(); return frame;}
    if (!fresh || !kart_system::valid(c)) {
      // Allow message delivery order at STOP->HOST, but never reuse the previous command.
      if (phase_!=Phase::waiting_input || now-started_>0.15) {stop();}
      return frame;
    }
    if (phase_!=Phase::armed) {
      if (!kart_system::neutral(c)) {stop(); return frame;}
      frame.flags |= HOST_REQUEST; phase_=Phase::neutral_sent; return esc_.apply(frame);
    }
    frame.flags |= HOST_REQUEST;
    const auto corrected=kart_system::apply_trim(c,trim,manual);
    frame.steering_milli=static_cast<int>(std::lround(std::clamp(corrected.steering*scale+offset,-1.0,1.0)*1000));
    frame.throttle_milli=static_cast<int>(std::lround(corrected.throttle*1000));
    frame.reverse_milli=static_cast<int>(std::lround(corrected.reverse*1000));
    frame.brake_milli=static_cast<int>(std::lround(corrected.brake*1000));
    return esc_.apply(frame);
  }
  Phase phase() const {return phase_;}
private:
  Phase phase_{Phase::disarmed}; double started_{0};
  EscState esc_;
};
}
