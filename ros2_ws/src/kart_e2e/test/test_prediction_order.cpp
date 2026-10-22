#include "kart_e2e/prediction_order.hpp"
#include "kart_e2e/control_gate.hpp"
#include <cassert>
#include <string>

int main() {
  for (const std::string policy : {"observe", "drop"}) {
    kart_e2e::PredictionOrder order(policy);
    assert(order.inspect(100).keep);
    assert(order.inspect(110).reason == nullptr);
    auto old = order.inspect(105);
    assert(std::string(old.reason) == "out_of_order");
    assert(old.previous == 110);
    assert(old.keep == (policy == "observe"));
    // An earlier arrival must not lower the high-water mark.
    assert(order.inspect(106).previous == 110);
    auto duplicate = order.inspect(110);
    assert(std::string(duplicate.reason) == "duplicate_stamp");
    assert(duplicate.keep == (policy == "observe"));
    auto invalid = order.inspect(0);
    assert(std::string(invalid.reason) == "invalid_stamp");
    assert(invalid.keep == (policy == "observe"));
    assert(order.inspect(120).keep);
    assert(order.backwards() == 2 && order.duplicates() == 1 && order.invalid() == 1);
    order.reset(); // mode transition / ROS clock epoch
    assert(order.inspect(10).reason == nullptr);
    assert(order.backwards() == 2);
  }
  bool threw = false;
  try { kart_e2e::PredictionOrder bad("typo"); }
  catch (const std::invalid_argument &) { threw = true; }
  assert(threw);

  // Monitoring preserves existing acceptance. Dropping keeps the last good
  // prediction without rejuvenating its stamp; normal expiry still stops.
  for (const std::string policy : {"observe", "drop"}) {
    kart_e2e::PredictionOrder order(policy);
    kart_e2e::ControlGate gate(true, .1, .2, .15, .3);
    gate.step(1., 1, 1., {}, 0.);
    order.inspect(1100000000);
    float prediction = .2f;
    double stamp = 1.10;
    assert(!gate.step(1.11, 1, 1.11, {prediction}, stamp).stop);
    if (order.inspect(1090000000).keep) { prediction = -.2f; stamp = 1.09; }
    auto output = gate.step(1.12, 1, 1.12, {prediction}, stamp);
    assert(!output.stop);
    assert((output.steering > 0) == (policy == "drop"));
    assert(gate.step(1.30, 1, 1.30, {prediction}, stamp).stop);
  }
}
