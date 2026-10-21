#include "kart_e2e/latency_tracker.hpp"
#include <cassert>
#include <cmath>
using kart_e2e::LatencyTracker;
int main() {
  LatencyTracker tracker(2);
  assert(tracker.input(100, 1000000, true));
  auto p = tracker.observe(100, 3500000, false);
  assert(p && p->first && p->warmup && std::abs(p->ms-2.5)<1e-10);
  auto c = tracker.observe(100, 6000000, true);
  assert(c && c->first && c->ms == 5);
  assert(!tracker.observe(100, 7000000, true)->first);
  assert(!tracker.observe(999, 7000000, false)); // Never nearest-match.
  assert(!tracker.observe(100, 999999, false)); // No negative durations.
  assert(!tracker.input(100, 8000000, false)); // Reused stamp invalidates earlier input.
  assert(!tracker.observe(100, 9000000, false));
  assert(tracker.input(200, 10000000, false));
  assert(!tracker.observe(200, 11000000, false)->warmup);
  assert(tracker.input(300, 12000000, false));
  assert(tracker.input(400, 13000000, false));
  assert(!tracker.observe(200, 14000000, false)); // Bounded metadata, not unbounded buffering.
  assert(!tracker.input(250, 15000000, false)); // Regressed source clock excluded.
  assert(!tracker.input(0, 16000000, false));
}
