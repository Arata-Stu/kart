#include "kart_control/pure_pursuit.hpp"
#include <cassert>
#include <limits>
using namespace kart_control;
int main() {
  Settings c{0.25, 0.5, 0.8, 2, 1, 0.1, 1};
  std::vector<Point> straight{{0, 0, 1}, {1, 0, 1}, {2, 0, 1}, {3, 0, 0}};
  auto r = pursue(straight, false, {0, 0, 0}, c);
  assert(r.valid && std::abs(r.steering) < 1e-12 && r.speed == 1);
  r = pursue(straight, false, {0, -0.2, 0}, c);
  assert(r.valid && r.steering > 0);
  r = pursue(straight, false, {0, 0.2, 0}, c);
  assert(r.valid && r.steering < 0);
  assert(!pursue(straight, false, {0, 2, 0}, c).valid);
  assert(!pursue(straight, false, {0, 0, 3.14159265}, c).valid);
  r = pursue(straight, false, {3, 0, 0}, c);
  assert(r.valid && r.speed == 0);
  auto stop = straight;
  for (auto &p : stop)
    p.speed = 0;
  assert(pursue(stop, false, {0, 0, 0}, c).speed == 0);
  auto invalid = straight;
  invalid[1].speed = std::numeric_limits<double>::quiet_NaN();
  assert(!pursue(invalid, false, {0, 0, 0}, c).valid);
  invalid = straight;
  invalid[1] = invalid[0];
  assert(!pursue(invalid, false, {0, 0, 0}, c).valid);
  std::vector<Point> loop{{0, 0, 1}, {2, 0, 1}, {2, 2, 1}, {0, 2, 1}};
  r = pursue(loop, true, {0, 0.2, -1.570796327}, c);
  assert(r.valid && r.target_x > 0 && std::abs(r.target_y) < 1e-9);
  auto rotated = straight;
  for (auto &p : rotated) {
    double x = p.x;
    p.x = 10 - p.y;
    p.y = 20 + x;
  }
  r = pursue(rotated, false, {10, 20, 1.5707963267948966}, c);
  assert(r.valid && std::abs(r.steering) < 1e-10 && r.speed == 1);
  r = pursue(straight, false, {2.8, 0, 0}, c);
  assert(r.valid && r.speed < 0.3);
  c.wheelbase = 0;
  assert(!pursue(straight, false, {0, 0, 0}, c).valid);
}
