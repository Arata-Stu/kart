#include "kart_control/fixed_command.hpp"
#include <cassert>
#include <limits>
using namespace kart_control;

int main() {
  FixedSettings s; s.throttle=.12; s.steering=-.05;
  auto tick=[](FixedCommandGate &g,int mode,double t,bool competing=false) {
    g.observe(mode,100+t,100+t,t); return g.step(100+t,t,competing);
  };
  FixedCommandGate startup(s);
  assert(tick(startup,1,0).request_stop); // Restarting in AUTO cannot launch the car.
  assert(tick(startup,3,.1).throttle==0);
  assert(tick(startup,1,.2).throttle==0);
  assert(tick(startup,1,.3).throttle==0);
  assert(tick(startup,1,.4).throttle==0);
  auto active=tick(startup,1,.51);
  assert(active.throttle==.12 && active.steering==-.05 && active.brake==0);
  assert(tick(startup,2,.6).throttle==0); // MANUAL is not overridden.
  assert(tick(startup,1,.7).request_stop); // STOP is mandatory between runs.
  assert(tick(startup,3,.8).throttle==0);
  tick(startup,1,.9);tick(startup,1,1.0);tick(startup,1,1.1);
  assert(tick(startup,1,1.21).throttle==.12);
  assert(tick(startup,1,1.3,true).request_stop);
  assert(tick(startup,1,1.4).throttle==0); // Fault latch survives peer disappearance.
  tick(startup,3,1.5);tick(startup,1,1.6);tick(startup,1,1.7);tick(startup,1,1.8);
  assert(tick(startup,1,1.91).throttle==.12);
  assert(startup.step(102.3,2.3,false).request_stop); // Lost mode and delayed tick.
  assert(tick(startup,1,2.4).throttle==0);
  FixedCommandGate gap(s);
  tick(gap,3,0);tick(gap,1,.1);tick(gap,1,.2);tick(gap,1,.3);
  assert(tick(gap,1,.41).throttle==.12);
  // A new heartbeat must not hide the preceding receive gap.
  assert(tick(gap,1,1).request_stop);
  FixedCommandGate stale(s);
  tick(stale,3,0);tick(stale,1,.1);
  stale.observe(3,99,100.2,.2);
  assert(stale.step(100.2,.2,false).request_stop); // stale STOP cannot clear latch.
  FixedSettings timed=s;timed.duration_s=.2;
  FixedCommandGate limited(timed);
  tick(limited,3,0);tick(limited,1,.1);tick(limited,1,.2);tick(limited,1,.3);
  assert(tick(limited,1,.41).throttle==.12);
  assert(tick(limited,1,.61).request_stop);
  assert(tick(limited,1,.7).throttle==0);
  FixedSettings braking; braking.brake=.1;
  FixedCommandGate brake(braking);
  tick(brake,3,0);tick(brake,1,.1);tick(brake,1,.2);tick(brake,1,.3);
  assert(tick(brake,1,.41).brake==.1);
  for (int scenario=0; scenario<4; ++scenario) {
    auto invalid=s;
    if (scenario==0) invalid.throttle=.3;
    if (scenario==1) invalid.brake=.1;
    if (scenario==2) invalid.steering=std::numeric_limits<double>::quiet_NaN();
    if (scenario==3) invalid.entry_neutral_s=0;
    bool threw=false;
    try {FixedCommandGate bad(invalid);} catch(const std::invalid_argument &) {threw=true;}
    assert(threw);
  }
  FixedSettings partial=s; partial.fixed_axes="throttle";
  FixedCommandGate mixed(partial);
  assert(!mixed.configurable(100,0));
  tick(mixed,3,0);
  assert(mixed.configurable(100,0));
  assert(!mixed.configurable(101,1));
  auto drive=[&](int mode,double t) {
    mixed.teleop(.4,.8,0,0,100+t,100+t,t);
    return tick(mixed,mode,t);
  };
  drive(1,.1);drive(1,.2);drive(1,.3);
  auto m=drive(1,.41);
  assert(m.throttle==.12 && m.steering==.4 && m.reverse==0);
  assert(!mixed.configurable(100.41,.41));
  assert(tick(mixed,1,.6).request_stop); // Free-axis input expired.
  assert(drive(1,.61).request_stop); // Returning input cannot rearm.
  drive(3,.7);
  partial.fixed_axes="steering";partial.steering=-.3;
  mixed.configure(partial,100.7,.7);
  drive(1,.8);drive(1,.9);drive(1,1.0);
  m=drive(1,1.11);
  assert(m.steering==-.3 && m.throttle==.2);
  mixed.teleop(.4,0,0,.8,101.12,101.12,1.12);
  m=tick(mixed,1,1.12);assert(m.reverse==.2 && m.throttle==0);
  mixed.teleop(0,.1,.2,0,101.13,101.13,1.13);
  assert(tick(mixed,1,1.13).request_stop); // Invalid manual input.
  drive(3,1.2);
  partial.fixed_axes="both";partial.throttle=.17;
  mixed.configure(partial,101.2,1.2);
  drive(1,1.3);drive(1,1.4);drive(1,1.5);
  assert(drive(1,1.61).throttle==.17);

}
