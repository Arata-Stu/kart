#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace kart_system {
constexpr uint8_t AUTO = 1, MANUAL = 2, STOP = 3, PROPO = 4;
inline bool host_mode(uint8_t mode) {return mode == AUTO || mode == MANUAL;}
inline bool valid_mode(uint8_t mode) {return mode >= AUTO && mode <= PROPO;}
struct Command {double steering{0}, throttle{0}, brake{0}, reverse{0};};
inline bool valid(const Command & c) {
  return std::isfinite(c.steering) && std::isfinite(c.throttle) &&
    std::isfinite(c.brake) && std::isfinite(c.reverse) &&
    std::abs(c.steering) <= 1 && c.throttle >= 0 && c.throttle <= 1 &&
    c.reverse >= 0 && c.reverse <= 1 && c.brake >= 0 && c.brake <= 1 &&
    !(c.throttle > 0 && c.reverse > 0) &&
    !(c.brake > 0 && (c.throttle > 0 || c.reverse > 0));
}
inline bool neutral(const Command & c) {
  return valid(c) && std::abs(c.steering) <= 0.05 && c.throttle <= 0.01 &&
    c.reverse <= 0.01 && c.brake <= 0.01;
}
inline bool fresh(double now, double received, double timeout) {
  return std::isfinite(received) && now >= received && now - received <= timeout;
}
inline bool fresh_stamp(double now, double stamp, double timeout) {
  return std::isfinite(stamp) && stamp > 0 && stamp <= now + 0.05 && now - stamp <= timeout;
}
constexpr double NEVER = -std::numeric_limits<double>::infinity();

// A local latch, deliberately independent of periodic mode messages.
// Restart/fault requires observing STOP, then a NEW host-mode transition.
class Authority {
public:
  bool observe(uint8_t mode, double now, bool permit) {
    last_mode_at_ = now;
    const auto previous = mode_;
    mode_ = mode;
    if (mode == STOP) {active_ = false; saw_stop_ = true; return false;}
    if (!host_mode(mode)) {trip(); return false;}
    if (previous != mode) {
      active_ = previous == STOP && saw_stop_ && permit;
      saw_stop_ = false;
    }
    return active_;
  }
  void trip() {active_ = false; saw_stop_ = false;}
  bool check(double now, double timeout, bool healthy) {
    if (!fresh(now, last_mode_at_, timeout) || !healthy) {trip();}
    return active_;
  }
  bool active() const {return active_;}
  uint8_t mode() const {return mode_;}
private:
  uint8_t mode_{0};
  bool active_{false}, saw_stop_{false};
  double last_mode_at_{NEVER};
};

struct ControlTrim {
  double steering{0}, throttle{0};
  bool adjust(double steering_delta, double throttle_delta,
              double steering_limit, double throttle_limit) {
    if (!std::isfinite(steering_delta) || !std::isfinite(throttle_delta) ||
        std::abs(steering_delta)>0.050001 || std::abs(throttle_delta)>0.050001) {return false;}
    steering=std::clamp(steering+steering_delta,-steering_limit,steering_limit);
    throttle=std::clamp(throttle+throttle_delta,-throttle_limit,throttle_limit);
    return true;
  }
};
// Apply only after the board has acknowledged HOST authority. A throttle trim
// changes operating effort, never neutral and never the requested direction.
inline Command apply_trim(Command c, const ControlTrim & trim, bool manual) {
  c.steering=std::clamp(c.steering+trim.steering,-1.0,1.0);
  if (manual && c.brake==0) {
    const double effort=c.throttle-c.reverse;
    const double adjusted=effort>0 ? std::clamp(effort+trim.throttle,0.0,1.0) :
      effort<0 ? std::clamp(effort+trim.throttle,-1.0,0.0) : 0.0;
    c.throttle=std::max(0.0,adjusted); c.reverse=std::max(0.0,-adjusted);
  }
  return c;
}

// Canonical kart_joy input: right-positive steering, resting triggers = 0.
inline Command teleop(double axis, double forward, double reverse, bool brake,
                      double deadzone, double steering_scale, double speed_scale) {
  Command c;
  c.steering = std::abs(axis) <= deadzone ? 0 :
    -std::copysign((std::abs(axis) - deadzone) / (1 - deadzone), axis) * steering_scale;
  if (brake) {c.brake = 1;}
  else {
    // ESC input: R2 positive, L2 negative (brake/reverse determined by ESC).
    const double signed_throttle = ((forward <= 0.01 ? 0 : forward) -
      (reverse <= 0.01 ? 0 : reverse)) * speed_scale;
    c.throttle = std::max(0.0, signed_throttle);
    c.reverse = std::max(0.0, -signed_throttle);
  }
  return c;
}
}  // namespace kart_system
