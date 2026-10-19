#include "kart_e2e/control_gate.hpp"
#include <cassert>
#include <limits>
using kart_e2e::ControlGate;
void arm(ControlGate &g) {
  assert(!g.step(10, 3, 10, {}, 0).stop);
  const auto d = g.step(11, 1, 11, {}, 0);
  assert(d.throttle == 0 && !d.stop);
}
int main() {
  ControlGate fixed(true, .12, .2, .15, .3);
  arm(fixed);
  auto d = fixed.step(11.11, 1, 11.1, {.4f}, 11.1);
  assert(std::abs(d.steering - .4) < 1e-6 && d.throttle == .12);
  assert(fixed.step(11.12, 3, 11.12, {.4f}, 11.1).throttle == 0);
  // The model's throttle output must not replace the fixed E2E demand.
  ControlGate forced(false, .12, .2, .15, .3, true);
  arm(forced);
  auto fixed_joint = forced.step(11.11, 1, 11.1, {-.8f, .9f}, 11.1);
  assert(std::abs(fixed_joint.steering + .8) < 1e-6 && fixed_joint.throttle == .12);
  assert(forced.step(11.5, 1, 11.5, {0, .1f}, 11.1).stop);
  ControlGate joint(false, 0, .2, .15, .3);
  arm(joint);
  assert(joint.step(11.11, 1, 11.1, {-.8f, .9f}, 11.1).throttle == .2);
  assert(joint.step(11.5, 1, 11.5, {0, .1f}, 11.1).stop);
  assert(joint.step(11.6, 1, 11.6, {0, .1f}, 11.6).stop);
  assert(!joint.step(12, 3, 12, {}, 0).stop);
  for (auto p : {std::vector<float>{std::numeric_limits<float>::quiet_NaN()},
                 std::vector<float>{2}, std::vector<float>{},
                 std::vector<float>{0, .1f}}) {
    ControlGate g(true, .1, .2, .15, .3);
    arm(g);
    assert(g.step(11.11, 1, 11.1, p, 11.1).stop);
  }
  for (double stamp : {12., 10.99}) {
    ControlGate g(true, 0, .2, .15, .3);
    arm(g);
    assert(g.step(11.11, 1, 11.1, {0}, stamp).stop);
  }
  // RealSense trace: a valid steer-only prediction stamped 2.1ms in the future.
  for (double lead : {0., .0021, .005}) {
    ControlGate g(true, .1, .2, .15, .3);
    arm(g);
    const auto result = g.step(11.11, 1, 11.1, {.337598f}, 11.11 + lead);
    assert(!result.stop && result.throttle == .1);
    assert(std::abs(result.steering - .337598) < 1e-6);
  }
  for (double stamp : {11.11 + .005001, 11.11 - .150001,
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    ControlGate g(true, .1, .2, .15, .3);
    // Enter AUTO earlier so the stale case tests expiry, not pre-entry rejection.
    arm(g);
    assert(!g.step(11.2, 1, 11.2, {0}, 11.2).stop);
    const auto image = std::isfinite(stamp) ? stamp + .2 : stamp;
    assert(g.step(11.31, 1, 11.3, {0}, image).stop);
    assert(g.step(11.32, 1, 11.32, {0}, 11.32).stop); // Still latched.
  }
  ControlGate strict(true, .1, .2, .15, .3, false, 0.);
  arm(strict);
  assert(strict.step(11.11, 1, 11.1, {0}, 11.1121).stop);
  for (double tolerance : {-.001, .051, std::numeric_limits<double>::quiet_NaN()}) {
    bool threw = false;
    try {ControlGate g(true, .1, .2, .15, .3, false, tolerance);}
    catch (const std::invalid_argument &) {threw = true;}
    assert(threw);
  }
  ControlGate competitor(true, 0, .2, .15, .3);
  arm(competitor);
  assert(competitor.step(11.11, 1, 11.1, {0}, 11.1, true).stop);
  ControlGate stale(true, 0, .2, .15, .3);
  arm(stale);
  assert(stale.step(12, 1, 11, {0}, 12).stop);
  for (double bad : {-.1, .3, std::numeric_limits<double>::quiet_NaN()}) {
    bool threw = false;
    try {
      ControlGate g(true, bad, .2, .15, .3);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    assert(threw);
  }
}
