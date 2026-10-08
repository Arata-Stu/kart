#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kart_control {
struct PidSettings {
  double kp{0}, ki{0}, kd{0}, kff{0};
  double integral_limit{0.2}, derivative_tau{0.1}, throttle_limit{0.2},
      brake_limit{0};
  bool operator==(const PidSettings &b) const {
    return kp == b.kp && ki == b.ki && kd == b.kd && kff == b.kff &&
           integral_limit == b.integral_limit &&
           derivative_tau == b.derivative_tau &&
           throttle_limit == b.throttle_limit && brake_limit == b.brake_limit;
  }
};
inline void validate(const PidSettings &c) {
  for (double v : {c.kp, c.ki, c.kd, c.kff, c.integral_limit, c.derivative_tau,
                   c.throttle_limit, c.brake_limit})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("PID values must be finite and nonnegative");
  if (c.throttle_limit > 1 || c.brake_limit > 1 || c.integral_limit > 1)
    throw std::invalid_argument("Normalized limits must be <= 1");
}
struct PidOutput {
  double throttle{}, brake{}, p{}, i{}, d{}, ff{};
};
class SpeedPid {
public:
  void reset() {
    integral_ = 0;
    derivative_ = 0;
    previous_ = 0;
    initialized_ = false;
  }
  PidOutput step(double target, double measured, double dt,
                 const PidSettings &c) {
    validate(c);
    if (!std::isfinite(target) || !std::isfinite(measured) ||
        !std::isfinite(dt) || target < 0 || dt <= 0 || dt > 0.2)
      throw std::invalid_argument("Invalid speed or PID dt");
    const double error = target - measured;
    // Derivative on measurement avoids a target-speed step causing derivative
    // kick.
    double raw = initialized_ ? -(measured - previous_) / dt : 0;
    derivative_ += dt / (c.derivative_tau + dt) * (raw - derivative_);
    previous_ = measured;
    initialized_ = true;
    PidOutput out;
    out.p = c.kp * error;
    out.d = c.kd * derivative_;
    out.ff = c.kff * target;
    double candidate = std::clamp(integral_ + c.ki * error * dt,
                                  -c.integral_limit, c.integral_limit);
    double trial = out.p + candidate + out.d + out.ff;
    // Conditional integration: retain only changes that do not deepen
    // saturation.
    if (!((trial > c.throttle_limit && error > 0) ||
          (trial < -c.brake_limit && error < 0)))
      integral_ = candidate;
    out.i = integral_;
    double u = std::clamp(out.p + out.i + out.d + out.ff, -c.brake_limit,
                          c.throttle_limit);
    out.throttle = std::max(0.0, u);
    out.brake = std::max(0.0, -u);
    return out;
  }

private:
  double integral_{0}, derivative_{0}, previous_{0};
  bool initialized_{false};
};
} // namespace kart_control
