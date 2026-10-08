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
