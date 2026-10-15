#pragma once
#include <cmath>

namespace kart_control {
// Explicit clock input permits tests independent of ROS and host execution speed.
struct TickDelta {
  double dt{0};
  bool advanced{false}, jumped{false};
};
class ControlTime {
public:
  TickDelta advance(double time) {
    if (!std::isfinite(time) || time < 0) return {0, false, true};
    if (!initialized_ && time == 0) return {}; // /clock not received yet.
    if (!initialized_) {
      previous_ = time; initialized_ = true;
      return {};
    }
    const double dt = time - previous_;
    previous_ = time;
    return {dt, dt > 0, dt < 0};
  }
private:
  double previous_{0};
  bool initialized_{false};
};
inline double receive_limit(double data_timeout, double wall_timeout) {
  return wall_timeout > 0 ? wall_timeout : data_timeout;
}
} // namespace kart_control
