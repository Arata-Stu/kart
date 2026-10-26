#pragma once
#include <algorithm>
#include <cmath>
#include <optional>

namespace kart_control {
struct LookaheadSettings {
  bool interpolated{false};
  double low_m{0.8}, high_m{2.0};
};
inline bool valid_lookahead(LookaheadSettings c) {
  return std::isfinite(c.low_m) && c.low_m > 0 &&
      std::isfinite(c.high_m) && c.high_m >= c.low_m;
}
inline std::optional<double> lookahead_distance(double fixed, double speed,
                                               LookaheadSettings c) {
  if (!valid_lookahead(c) || !std::isfinite(fixed) || fixed <= 0) return {};
  if (!c.interpolated) return fixed;
  if (!std::isfinite(speed)) return {};
  // The two speed anchors are fixed at 1 m/s and 5 m/s. Clamp outside them.
  const double ratio = std::clamp((std::abs(speed) - 1.0) / 4.0, 0.0, 1.0);
  return c.low_m + (c.high_m - c.low_m) * ratio;
}
} // namespace kart_control
