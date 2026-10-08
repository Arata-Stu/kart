#include "kart_system/control_core.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace kart_system;
int main() {
  Authority a;
  assert(!a.observe(MANUAL,1,true)); // Startup active state cannot arm.
  a.observe(STOP,2,true);
  assert(!a.observe(MANUAL,3,false)); // Non-neutral/missing input.
  assert(!a.observe(MANUAL,4,true)); // Healthy input alone cannot rearm.
  a.observe(STOP,5,true); assert(a.observe(MANUAL,6,true));
  assert(a.check(6.1,0.3,true));
  assert(!a.check(6.4,0.3,true)); // Mode publisher loss.
  assert(!a.observe(MANUAL,6.5,true));
  a.observe(STOP,7,true); assert(a.observe(AUTO,8,true));
  assert(!a.observe(MANUAL,8.1,true)); // Requires intermediate STOP.
  a.observe(STOP,9,true); assert(a.observe(AUTO,10,true));
  assert(!a.check(10.1,0.3,false)); // Command timeout/invalid input.
  assert(!a.observe(AUTO,10.2,true));
  a.observe(PROPO,11,true); assert(!a.observe(AUTO,12,true));
  a.observe(STOP,13,true); assert(a.observe(AUTO,14,true));
  assert(!valid({0,1,0,1})); assert(!valid({0,0.1,1,0}));
  assert(!valid({std::numeric_limits<double>::quiet_NaN(),0,0,0}));
  assert(!valid({0,-0.1,0,0})); assert(neutral({}));
  assert(!fresh(9,10,0.2)); assert(!fresh(10,NEVER,0.2));
  assert(!fresh_stamp(10,0,0.15)); assert(!fresh_stamp(10,9.8,0.15));
  assert(!fresh_stamp(10,10.1,0.15)); assert(fresh_stamp(10,9.9,0.15));
  auto c=teleop(-1,0.5,0,false,0.08,1,0.3);
  assert(c.steering==1 && c.throttle==0.15 && c.reverse==0);
  assert(neutral(teleop(0.07,0,0,false,0.08,1,0.3)));
  assert(neutral(teleop(0,1,1,false,0.08,1,0.3)));
  c=teleop(0,0.2,0.8,false,0.08,1,0.3);
  assert(c.throttle==0 && std::abs(c.reverse-0.18)<1e-12 && c.brake==0);
  c=teleop(0,0.8,0.2,false,0.08,1,0.3);
  assert(c.reverse==0 && std::abs(c.throttle-0.18)<1e-12);
  c=teleop(0,0,1,false,0.08,1,0.3);
  assert(c.reverse==0.3 && c.throttle==0 && c.brake==0);
  c=teleop(0,1,0,true,0.08,1,0.3); assert(c.brake==1 && c.throttle==0);
  ControlTrim trim;
  assert(trim.adjust(0.01,0.03,0.25,0.1));
  c=apply_trim({0,0.2,0,0},trim,true);
  assert(std::abs(c.throttle-0.23)<1e-12 && c.steering==0.01 && valid(c));
  c=apply_trim({0,0,0,0.2},trim,true);
  assert(std::abs(c.reverse-0.17)<1e-12 && c.throttle==0 && valid(c));
  c=apply_trim({},trim,true); assert(c.throttle==0 && c.reverse==0);
  c=apply_trim({0,0.2,0,0},trim,false); assert(c.throttle==0.2 && c.steering==0.01);
  c=apply_trim({0,0,1,0},trim,true); assert(c.brake==1 && c.throttle==0 && c.reverse==0);
  c=apply_trim({0,0,0,0.01},trim,true); assert(c.reverse==0 && c.throttle==0); // Cannot reverse direction.
  for(int i=0;i<100;++i) {assert(trim.adjust(0.01,0.01,0.25,0.1));}
  assert(trim.steering==0.25 && trim.throttle==0.1);
  assert(!trim.adjust(0.01,std::numeric_limits<double>::quiet_NaN(),0.25,0.1));
  assert(trim.steering==0.25 && trim.throttle==0.1); // Atomic rejection.
  assert(!trim.adjust(1,0,0.25,0.1));
  for(int i=0;i<100;++i) {assert(trim.adjust(-0.01,-0.01,0.25,0.1));}
  assert(trim.steering==-0.25 && trim.throttle==-0.1);
  c=apply_trim({0,0.01,0,0},trim,true); assert(c.throttle==0 && c.reverse==0);
  assert(valid(apply_trim({-1,0,0,1},trim,true))); // Saturation at bounds.
  std::cout << "control core: passed\n";
}
