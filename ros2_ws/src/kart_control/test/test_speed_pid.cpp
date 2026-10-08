#include "kart_control/speed_pid.hpp"
#include <cassert>
#include <limits>
using namespace kart_control;
int main() {
  SpeedPid pid;
  PidSettings c;
  c.kp = 0.2;
  c.ki = 0.1;
  c.throttle_limit = 0.8;
  auto r = pid.step(1, 0, 0.1, c);
  assert(std::abs(r.throttle - 0.21) < 1e-9);
  pid.reset();
  r = pid.step(1, 0, 0.1, c);
  assert(std::abs(r.i - 0.01) < 1e-9);
  c.throttle_limit = 0.1;
  pid.reset();
  for (int n = 0; n < 1000; n++)
    r = pid.step(10, 0, 0.1, c);
  assert(r.throttle == 0.1 && r.i == 0);
  r = pid.step(0, 1, 0.1, c);
  assert(r.throttle == 0 && r.brake == 0);
  c.brake_limit = 0.3;
  r = pid.step(0, 1, 0.1, c);
  assert(r.brake > 0 && r.throttle == 0);
  c.kp = 0;
  c.ki = 0;
  c.kd = 0.1;
  pid.reset();
  pid.step(0, 0, 0.1, c);
  r = pid.step(10, 0, 0.1, c);
  assert(r.d == 0); // no derivative kick
  r = pid.step(10, 1, 0.1, c);
  assert(r.d < 0);
  c.kd = 0;
  c.kff = 0.05;
  pid.reset();
  r = pid.step(1, 1, 0.1, c);
  assert(std::abs(r.throttle - 0.05) < 1e-9);
  bool threw = false;
  try {
    pid.step(1, 0, 0.3, c);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);
  c.kp = std::numeric_limits<double>::quiet_NaN();
  threw = false;
  try {
    validate(c);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  assert(threw);

  // Closed-loop synthetic first-order plant, not a vehicle calibration.
  PidSettings sim;
  sim.kp = 0.8;
  sim.ki = 0.4;
  sim.throttle_limit = 1.0;
  sim.integral_limit = 1.0;
  pid.reset();
  double speed = 0.0;
  for (int n = 0; n < 3000; ++n) {
    auto control = pid.step(1.0, speed, 0.01, sim);
    speed += 0.01 * (2.0 * control.throttle - speed) / 0.5;
    assert(control.throttle >= 0 && control.throttle <= 1 &&
           control.brake == 0);
  }
  assert(std::abs(speed - 1.0) < 0.01);
}
