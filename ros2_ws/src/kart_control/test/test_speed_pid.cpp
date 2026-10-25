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

  // Accepted steady-speed proxies: table FF supersedes the proportional kff.
  PidSettings table;
  table.ff_speed_mps = {1.41, 1.90, 2.27, 2.67, 3.05, 3.35,
                        3.69, 3.83, 4.14, 4.34, 4.63, 4.74};
  table.ff_throttle = {.100, .125, .150, .175, .200, .225,
                       .250, .275, .300, .325, .350, .375};
  table.throttle_limit = .4;
  table.kff = 10; // Not added to lookup FF.
  validate(table);
  for (size_t n=0; n<table.ff_speed_mps.size(); ++n) {
    pid.reset();
    r = pid.step(table.ff_speed_mps[n], table.ff_speed_mps[n], .02, table);
    assert(std::abs(r.ff - table.ff_throttle[n]) < 1e-12);
    assert(r.throttle == r.ff);
  }
  assert(std::abs(feedforward(3., table) - .1967105263157895) < 1e-12);
  assert(std::abs(feedforward(.705, table) - .05) < 1e-12);
  assert(feedforward(0, table) == 0 && feedforward(10, table) == .375);
  table.kp = .02;
  pid.reset();
  r = pid.step(3.05, 2.05, .02, table);
  assert(std::abs(r.throttle - .22) < 1e-12);
  r = pid.step(3.05, 4.05, .02, table);
  assert(std::abs(r.throttle - .18) < 1e-12);
  table.throttle_rise_rate = .1;
  table.throttle_fall_rate = .2;
  table.ki = .1;
  pid.reset();
  r = pid.step(4.74, 0, .1, table);
  assert(std::abs(r.throttle-.01) < 1e-12 && r.i == 0);
  r = pid.step(4.74, 0, .1, table);
  assert(std::abs(r.throttle-.02) < 1e-12 && r.i == 0);
  // Fall slew is exercised with a positive lower target; zero bypasses it.
  table.kp = table.ki = 0;
  for (int n=0; n<40; ++n) r = pid.step(4.74, 4.74, .1, table);
  assert(std::abs(r.throttle-.375) < 1e-12);
  pid.reset(true); // Live gain changes reset feedback without restarting slew at zero.
  r = pid.step(4.74, 4.74, .1, table);
  assert(std::abs(r.throttle-.375) < 1e-12);
  r = pid.step(.705, .705, .1, table);
  assert(std::abs(r.throttle-.355) < 1e-12);
  r = pid.step(0, 2, .1, table);
  assert(r.throttle == 0 && r.brake == 0);
  r = pid.step(4.74, 4.74, .1, table);
  assert(std::abs(r.throttle-.01) < 1e-12);
  pid.reset();
  r = pid.step(4.74, 4.74, .02, table);
  assert(std::abs(r.throttle-.002) < 1e-12);
  // Brake must not overlap the throttled ramp-down.
  table.brake_limit = .3; table.kp = .2;
  r = pid.step(.1, 5, .1, table);
  assert(r.throttle == 0 && r.brake > 0);
  for (int scenario=0; scenario<8; ++scenario) {
    auto invalid = table;
    if (scenario==0) invalid.ff_throttle.pop_back();
    if (scenario==1) invalid.ff_speed_mps[1]=invalid.ff_speed_mps[0];
    if (scenario==2) invalid.ff_throttle[1]=.05;
    if (scenario==3) invalid.ff_speed_mps[0]=0;
    if (scenario==4) invalid.ff_throttle[0]=1.1;
    if (scenario==5) invalid.ff_throttle[0]=std::numeric_limits<double>::quiet_NaN();
    if (scenario==6) invalid.throttle_rise_rate=-1;
    if (scenario==7) invalid.throttle_fall_rate=std::numeric_limits<double>::infinity();
    threw=false;
    try {validate(invalid);} catch(const std::invalid_argument &) {threw=true;}
    assert(threw);
  }

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
