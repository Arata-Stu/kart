#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace kart_control {
struct Point {
  double x, y, speed;
};
struct Pose {
  double x, y, yaw;
};
struct Settings {
  double wheelbase, steering_limit, lookahead, max_speed, max_error,
      goal_tolerance, deceleration;
};
struct Result {
  double steering{}, speed{}, target_x{}, target_y{};
  bool valid{};
};

inline Result pursue(const std::vector<Point> &line, bool closed, Pose pose,
                     Settings c) {
  for (double v : {c.wheelbase, c.steering_limit, c.lookahead, c.max_speed,
                   c.max_error, c.goal_tolerance, c.deceleration})
    if (!std::isfinite(v) || v <= 0)
      return {};
  for (double v : {pose.x, pose.y, pose.yaw})
    if (!std::isfinite(v))
      return {};
  if (line.size() < (closed ? 3U : 2U))
    return {};
  const size_t count = closed ? line.size() : line.size() - 1;
  std::vector<double> lengths(count);
  double total = 0, best = std::numeric_limits<double>::infinity(),
         fraction = 0;
  size_t nearest = 0;
  for (const auto &p : line)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.speed) ||
        p.speed < 0)
      return {};
  for (size_t i = 0; i < count; ++i) {
    const auto &a = line[i];
    const auto &b = line[(i + 1) % line.size()];
    double dx = b.x - a.x, dy = b.y - a.y, len = std::hypot(dx, dy);
    if (len < 1e-6)
      return {}; // Ambiguous duplicate points are rejected.
    lengths[i] = len;
    total += len;
    double t = std::clamp(
        ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / (len * len), 0.0, 1.0);
    double distance = std::hypot(a.x + t * dx - pose.x, a.y + t * dy - pose.y);
    if (distance < best) {
      best = distance;
      nearest = i;
      fraction = t;
    }
  }
  if (best > c.max_error || (closed && total <= c.lookahead))
    return {};
  double remaining = lengths[nearest] * (1 - fraction);
  for (size_t i = nearest + 1; i < count; ++i)
    remaining += lengths[i];
  if (!closed && remaining <= c.goal_tolerance)
    return {0, 0, line.back().x, line.back().y, true};
  // Travel forward along arc length from the nearest segment projection.
  double distance = c.lookahead, start = fraction;
  size_t segment = nearest;
  double requested =
      line[nearest].speed +
      (line[(nearest + 1) % line.size()].speed - line[nearest].speed) *
          fraction;
  for (size_t step = 0; step <= count; ++step) {
    double available = lengths[segment] * (1 - start);
    const auto &a = line[segment];
    const auto &b = line[(segment + 1) % line.size()];
    const bool last = !closed && segment + 1 == count;
    if (distance <= available || last) {
      double t = std::min(1.0, start + distance / lengths[segment]);
      double x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y);
      double dx = x - pose.x, dy = y - pose.y;
      double forward = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
      double lateral = -std::sin(pose.yaw) * dx + std::cos(pose.yaw) * dy;
      if (forward <= 1e-6)
        return {};
      double steering =
          std::atan2(2 * c.wheelbase * lateral, dx * dx + dy * dy);
      double speed = std::min(c.max_speed, requested);
      // Anticipate lower speed targets, including explicit zeros, without
      // skipping them.
      double travelled = lengths[nearest] * (1 - fraction);
      for (size_t n = 0, j = nearest; n < count && travelled <= c.lookahead;
           n++, j = (j + 1) % count) {
        speed =
            std::min(speed, std::sqrt(line[(j + 1) % line.size()].speed *
                                          line[(j + 1) % line.size()].speed +
                                      2 * c.deceleration * travelled));
        if (!closed && j + 1 == count)
          break;
        travelled += lengths[(j + 1) % count];
      }
      if (!closed)
        speed = std::min(
            speed, std::sqrt(2 * c.deceleration *
                             std::max(0.0, remaining - c.goal_tolerance)));
      return {std::clamp(steering, -c.steering_limit, c.steering_limit), speed,
              x, y, true};
    }
    distance -= available;
    segment = (segment + 1) % count;
    start = 0;
  }
  return {};
}
} // namespace kart_control
