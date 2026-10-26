#pragma once
#include <algorithm>
#include <cmath>
#include <optional>

namespace kart_control {
struct LookaheadSettings {
  double time_s{0}, min_m{0.4}, max_m{2.0};
};
inline bool valid_lookahead(LookaheadSettings c) {
  return std::isfinite(c.time_s) && c.time_s >= 0 &&
      std::isfinite(c.min_m) && c.min_m > 0 &&
      std::isfinite(c.max_m) && c.max_m >= c.min_m;
}
inline std::optional<double> lookahead_distance(double fixed, double speed,
                                               LookaheadSettings c) {
  if (!valid_lookahead(c) || !std::isfinite(fixed) || fixed <= 0) return {};
  if (c.time_s == 0) return fixed; // Fixed mode requires no velocity estimate.
  if (!std::isfinite(speed)) return {};
  const double distance = std::abs(speed) * c.time_s;
  // Very large but finite speed/time must saturate, never wrap or return NaN.
  return std::clamp(distance, c.min_m, c.max_m);
}
} // namespace kart_control
