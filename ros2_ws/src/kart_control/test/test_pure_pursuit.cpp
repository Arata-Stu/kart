#include "kart_control/pure_pursuit.hpp"
#include "kart_control/lookahead.hpp"
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
  // The displayed target is selected by the controller: arc length from the
  // nearest projection is L, while vehicle-to-target distance may be larger.
  assert(std::abs(r.target_x - c.lookahead) < 1e-12);
  assert(std::abs(r.target_y) < 1e-12);
  assert(std::hypot(r.target_x, r.target_y + 0.2) > c.lookahead);
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
  // Near an open-line end the actual reference is clamped to the endpoint.
  assert(std::abs(r.target_x - 3) < 1e-12);
  assert(std::abs(r.target_y) < 1e-12);
  assert(std::hypot(r.target_x - 2.8, r.target_y) < c.lookahead);
  {
    LookaheadSettings adaptive{0, .4, 2};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    assert(lookahead_distance(.8, nan, adaptive)==.8); // Fixed mode ignores unavailable speed.
    adaptive.time_s=.6;
    assert(lookahead_distance(.8, 0, adaptive)==.4);
    assert(std::abs(*lookahead_distance(.8, 1.5, adaptive)-.9)<1e-12);
    assert(lookahead_distance(.8, 10, adaptive)==2.);
    assert(lookahead_distance(.8, -10, adaptive)==2.); // Speed magnitude, never negative lookahead.
    assert(!lookahead_distance(.8, nan, adaptive));
    assert(!lookahead_distance(.8, std::numeric_limits<double>::infinity(), adaptive));
    assert(!lookahead_distance(.8, 1, {-1,.4,2}));
    assert(!lookahead_distance(.8, 1, {1,2,.4}));
    assert(!lookahead_distance(.8, 1, {1,0,2}));
    assert(!lookahead_distance(.8, 1, {nan,.4,2}));
    assert(!lookahead_distance(.8, 1, {1,.4,nan}));
    assert(lookahead_distance(.8, std::numeric_limits<double>::max(), {2,.4,2})==2.);
    // Speed changes move the actual reference point along the same path.
    auto short_config=c, long_config=c;
    short_config.lookahead=*lookahead_distance(.8, .5, adaptive);
    long_config.lookahead=*lookahead_distance(.8, 2, adaptive);
    auto short_result=pursue(straight,false,{0,0,0},short_config);
    auto long_result=pursue(straight,false,{0,0,0},long_config);
    assert(short_result.valid && long_result.valid);
    assert(std::abs(short_result.target_x-.4)<1e-12);
    assert(std::abs(long_result.target_x-1.2)<1e-12);
    std::vector<Point> bend{{0,0,1},{1,0,1},{1,2,1}};
    auto curved=pursue(bend,false,{0,0,0},long_config);
    assert(curved.valid && std::abs(curved.target_x-1)<1e-12 &&
           std::abs(curved.target_y-.2)<1e-12); // Arc length, not a circle intersection.
    long_config.lookahead=9;
    assert(!pursue(loop,true,{0,0,0},long_config).valid); // Existing short-loop gate remains.
  }
  c.wheelbase = 0;
  assert(!pursue(straight, false, {0, 0, 0}, c).valid);
}
