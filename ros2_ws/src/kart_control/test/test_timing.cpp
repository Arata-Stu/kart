#include "kart_control/timing.hpp"
#include "kart_control/speed_pid.hpp"
#include <cassert>
#include <limits>
using namespace kart_control;
int main() {
  ControlTime clock;
  assert(!clock.advance(0).advanced);
  assert(!clock.advance(100).advanced); // Late subscription: no 100 s PID dt.
  auto t = clock.advance(100.02);
  assert(t.advanced && !t.jumped && std::abs(t.dt - .02) < 1e-9);
  assert(!clock.advance(100.02).advanced); // Paused / duplicate clock.
  assert(clock.advance(1).jumped); // Reset must invalidate the control state.
  assert(clock.advance(std::numeric_limits<double>::quiet_NaN()).jumped);
  assert(clock.advance(2).dt > .2); // Forward discontinuity still violates PID deadline.
  assert(receive_limit(.15, 0) == .15); // Physical defaults are unchanged.
  assert(receive_limit(.15, 5) == 5);

  // Identical simulated measurements at 1x and 25x slower host execution.
  // Host delay must not inflate PID integration or change derivative response.
  ControlTime normal, slow;
  SpeedPid first, second;
  PidSettings c;
  c.kp=.1; c.ki=.05; c.kd=.01; c.throttle_limit=1; c.brake_limit=1;
  normal.advance(10); slow.advance(10);
  double wall=0;
  for (int n=1; n<=100; ++n) {
    const double sim=10+n*.02, measured=.002*n;
    wall += .5;
    auto a=normal.advance(sim), b=slow.advance(sim);
    assert(wall > n*.02);
    auto x=first.step(.7, measured, a.dt, c);
    auto y=second.step(.7, measured, b.dt, c);
    assert(x.i == y.i && x.d == y.d && x.throttle == y.throttle);
    assert(!slow.advance(sim).advanced); // No extra integration while /clock freezes.
  }
}
