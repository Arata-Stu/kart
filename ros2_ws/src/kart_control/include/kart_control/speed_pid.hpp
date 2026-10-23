#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace kart_control {
struct PidSettings {
  double kp{0}, ki{0}, kd{0}, kff{0};
  double integral_limit{0.2}, derivative_tau{0.1}, throttle_limit{0.2},
      brake_limit{0};
  std::vector<double> ff_speed_mps, ff_throttle;
  double throttle_rise_rate{0}, throttle_fall_rate{0};
  bool operator==(const PidSettings &b) const {
    return kp == b.kp && ki == b.ki && kd == b.kd && kff == b.kff &&
           integral_limit == b.integral_limit &&
           derivative_tau == b.derivative_tau &&
           throttle_limit == b.throttle_limit && brake_limit == b.brake_limit &&
           ff_speed_mps == b.ff_speed_mps && ff_throttle == b.ff_throttle &&
           throttle_rise_rate == b.throttle_rise_rate &&
           throttle_fall_rate == b.throttle_fall_rate;
  }
};
inline void validate(const PidSettings &c) {
  for (double v : {c.kp, c.ki, c.kd, c.kff, c.integral_limit, c.derivative_tau,
                   c.throttle_limit, c.brake_limit, c.throttle_rise_rate,
                   c.throttle_fall_rate})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("PID values must be finite and nonnegative");
  if (c.throttle_limit > 1 || c.brake_limit > 1 || c.integral_limit > 1)
    throw std::invalid_argument("Normalized limits must be <= 1");
  if (c.ff_speed_mps.size() != c.ff_throttle.size() ||
      (!c.ff_speed_mps.empty() && c.ff_speed_mps.size() < 2))
    throw std::invalid_argument("Feedforward arrays must have equal length >= 2, or both be empty");
  for (size_t i = 0; i < c.ff_speed_mps.size(); ++i) {
    if (!std::isfinite(c.ff_speed_mps[i]) || c.ff_speed_mps[i] <= 0 ||
        !std::isfinite(c.ff_throttle[i]) || c.ff_throttle[i] < 0 || c.ff_throttle[i] > 1 ||
        (i && (c.ff_speed_mps[i] <= c.ff_speed_mps[i-1] ||
               c.ff_throttle[i] < c.ff_throttle[i-1])))
      throw std::invalid_argument("Feedforward speeds must increase; throttles must be nondecreasing in [0,1]");
  }
}
// Below the measured range, interpolate from assumed (0,0); above it, hold
// the final throttle. Never extrapolate the final slope beyond measurements.
inline double feedforward(double target, const PidSettings &c) {
  if (c.ff_speed_mps.empty()) return c.kff * target;
  if (target <= 0) return 0;
  auto upper = std::upper_bound(c.ff_speed_mps.begin(), c.ff_speed_mps.end(), target);
  if (upper == c.ff_speed_mps.end()) return c.ff_throttle.back();
  const auto i = static_cast<size_t>(upper - c.ff_speed_mps.begin());
  const double v0 = i ? c.ff_speed_mps[i-1] : 0;
  const double u0 = i ? c.ff_throttle[i-1] : 0;
  return u0 + (c.ff_throttle[i] - u0) * (target - v0) / (c.ff_speed_mps[i] - v0);
}
struct PidOutput {
  double throttle{}, brake{}, p{}, i{}, d{}, ff{}, requested_throttle{};
};
class SpeedPid {
public:
  void reset() {
    integral_ = 0;
    derivative_ = 0;
    previous_ = 0;
    initialized_ = false;
    previous_throttle_ = 0;
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
    out.ff = feedforward(target, c);
    double candidate = std::clamp(integral_ + c.ki * error * dt,
                                  -c.integral_limit, c.integral_limit);
    double trial = out.p + candidate + out.d + out.ff;
    // Conditional integration: retain only changes that do not deepen
    // saturation.
    const double ceiling = c.throttle_rise_rate > 0
        ? std::min(c.throttle_limit, previous_throttle_ + c.throttle_rise_rate * dt)
        : c.throttle_limit;
    const double floor = c.throttle_fall_rate > 0
        ? std::min(ceiling, std::max(0.0, previous_throttle_ - c.throttle_fall_rate * dt)) : 0;
    if (!((trial > ceiling && error > 0) ||
          (trial < (floor > 0 ? floor : -c.brake_limit) && error < 0)))
      integral_ = candidate;
    out.i = integral_;
    double u = std::clamp(out.p + out.i + out.d + out.ff, -c.brake_limit,
                          c.throttle_limit);
    out.requested_throttle = target > 0 ? std::max(0.0, u) : 0;
    // Zero targets and braking bypass throttle slew: never delay neutral/STOP.
    out.throttle = target <= 0 || u < 0 ? 0 :
        std::min(c.throttle_limit, std::clamp(out.requested_throttle, floor, ceiling));
    out.brake = std::max(0.0, -u);
    previous_throttle_ = out.throttle;
    return out;
  }

private:
  double integral_{0}, derivative_{0}, previous_{0};
  double previous_throttle_{0};
  bool initialized_{false};
};
} // namespace kart_control
