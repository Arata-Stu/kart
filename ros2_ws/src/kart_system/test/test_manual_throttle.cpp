#include "kart_system/manual_throttle.hpp"
#include "kart_system/joy_controls.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace kart_system;
int main() {
  ManualFixedThrottle gate;
  assert(gate.valid_settings(0.3) && !gate.enabled());
  Command raw{0.4,0.25,0,0};
  gate.observe(MANUAL,10,10,1,true,true);
  auto result=gate.command(raw,false,10,1,true);
  assert(!result.stop && result.command.throttle==0.25);
  for (double value : {-0.1,0.31,std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    gate.throttle=value; assert(!gate.valid_settings(0.3));
  }
  gate.throttle=0.1;
  assert(gate.valid_settings(0.3) && gate.enabled());
  gate.trip();
  result=gate.command(raw,false,10,1,true);
  assert(result.stop && neutral(result.command)); // Startup MANUAL may not arm.

  double ros=20,wall=10;
  auto arm=[&]() {
    ros+=1; wall+=1;
    gate.observe(STOP,ros,ros,wall,true,true);
    ros+=0.1; wall+=0.1;
    gate.observe(MANUAL,ros,ros,wall,true,true);
    result=gate.command({},false,ros,wall,true);
    assert(!result.stop && neutral(result.command));
    for (int i=0;i<4;++i) {
      ros+=0.1; wall+=0.1;
      gate.observe(MANUAL,ros,ros,wall,true,true);
      result=gate.command({},false,ros,wall,true);
    }
    assert(gate.active() && result.command.throttle==gate.throttle);
  };
  arm();
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.steering==0.4 && result.command.throttle==0.1);
  raw.throttle=0; result=gate.command(raw,false,ros,wall,true);
  assert(result.command.throttle==0.1); // R2 released: fixed output remains.
  raw.throttle=0.3; result=gate.command(raw,false,ros,wall,true);
  assert(result.command.throttle==0.1); // R2 cannot override fixed value.
  result=gate.command({0.2,0,1,0},false,ros,wall,true);
  assert(result.stop && result.command.brake==1 && result.command.throttle==0);
  result=gate.command(raw,false,ros,wall,true);
  assert(result.stop && neutral(result.command)); // Brake release cannot resume.
  gate.observe(MANUAL,ros,ros,wall,true,true);
  assert(!gate.active()); // Periodic MANUAL never rearms after cancellation.
  arm();
  result=gate.command({0.2,0,0,0.3},true,ros,wall,true);
  assert(result.stop && neutral(result.command)); // L2 cancels; never reverse.
  arm(); result=gate.command(raw,true,ros,wall,true);
  assert(result.stop); // Both triggers: detect raw L2 even if teleop cancels it.
  arm(); assert(!gate.allowed(ros+0.4,wall+0.4,true));
  gate.observe(MANUAL,ros+0.4,ros+0.4,wall+0.4,true,true);
  assert(!gate.active()); // Mode freshness/recovery require a new STOP.
  arm(); assert(!gate.allowed(ros,wall,false)); // Joy/ready/deadman loss latches off.
  arm(); assert(!gate.allowed(ros-0.01,wall,true)); // Even a small ROS time reversal.
  arm(); assert(!gate.allowed(ros,wall+0.31,true)); // Receipt stale with paused ROS clock.
  arm(); result=gate.command({0,std::numeric_limits<double>::quiet_NaN(),0,0},false,ros,wall,true);
  assert(result.stop && neutral(result.command));
  arm(); gate.trip(); assert(!gate.active()); // Explicit STOP immediately removes authority.
  gate.observe(AUTO,ros,ros,wall,true,true);
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.throttle==raw.throttle); // AUTO auxiliary teleop stays raw.
  gate.observe(MANUAL,ros,ros,wall,true,true);
  assert(!gate.active()); // Direct AUTO -> MANUAL never arms.
  gate.throttle=0; arm();
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.throttle==0 && result.command.steering==raw.steering);

  ManualFixedThrottle nonneutral;
  nonneutral.throttle=0.1;
  nonneutral.observe(STOP,10,10,1,true,true);
  nonneutral.observe(MANUAL,10.1,10.1,1.1,true,false);
  assert(!nonneutral.active()); // External MANUAL request with axes held cannot arm.
  TeleopInput input;
  std::vector<float> axes(8,0);
  assert(input.can_arm(axes,false)); // Arming uses raw neutral, not injected throttle.
  axes[5]=0.2; assert(!input.can_arm(axes,false));

  std::cout<<"manual fixed throttle: passed\n";
}
