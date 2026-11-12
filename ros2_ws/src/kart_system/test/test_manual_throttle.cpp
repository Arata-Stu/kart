#include "kart_system/manual_throttle.hpp"
#include "kart_system/joy_controls.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <string>
using namespace kart_system;
int main() {
  {
    ManualFixedThrottle adjustable;
    assert(!adjustable.adjust(1,0.3,10,true)); // Disabled never becomes enabled by D-pad.
    adjustable.throttle=0.1;
    adjustable.observe(STOP,10,10,1,true,true);
    assert(adjustable.adjust(1,0.3,10,true) && std::abs(adjustable.throttle-0.101)<1e-9);
    assert(adjustable.adjust(-1,0.3,10,true) && std::abs(adjustable.throttle-0.1)<1e-9);
    assert(!adjustable.adjust(1,0.3,10,false));
    assert(!adjustable.adjust(1,0.3,10.31,true)); // Stale STOP.
    assert(!adjustable.adjust(1,0.3,9.9,true)); // Future STOP stamp.
    assert(!adjustable.adjust(0,0.3,10,true));
    assert(!adjustable.adjust(1,std::numeric_limits<double>::quiet_NaN(),10,true));
    for(int i=0;i<500;++i) adjustable.adjust(1,0.3,10,true);
    assert(adjustable.throttle==0.3);
    for(int i=0;i<500;++i) adjustable.adjust(-1,0.3,10,true);
    assert(adjustable.throttle==0);
    adjustable.adjust(1,0.3,10,true);
    adjustable.observe(AUTO,10.01,10.01,1.01,true,true);
    assert(!adjustable.adjust(1,0.3,10.01,true));
    adjustable.observe(AUTO,10.32,10.32,1.32,true,true);
    auto output=adjustable.command({},false,10.32,1.32,true);
    assert(!output.stop && std::abs(output.command.throttle-0.001)<1e-9);
    adjustable.observe(MANUAL,10.33,10.33,1.33,true,true);
    assert(!adjustable.adjust(-1,0.3,10.33,true));
  }
  ManualFixedThrottle gate;
  assert(gate.valid_settings(0.3) && !gate.enabled());
  Command raw{0.4,0.25,0,0};
  gate.observe(AUTO,10,10,1,true,true);
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
  assert(result.stop && neutral(result.command)); // Startup AUTO may not arm.

  double ros=20,wall=10;
  auto arm=[&]() {
    ros+=1; wall+=1;
    gate.observe(STOP,ros,ros,wall,true,true);
    ros+=0.1; wall+=0.1;
    gate.observe(AUTO,ros,ros,wall,true,true);
    result=gate.command({},false,ros,wall,true);
    assert(!result.stop && neutral(result.command));
    for (int i=0;i<4;++i) {
      ros+=0.1; wall+=0.1;
      gate.observe(AUTO,ros,ros,wall,true,true);
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
  gate.observe(AUTO,ros,ros,wall,true,true);
  assert(!gate.active()); // Periodic AUTO never rearms after cancellation.
  arm();
  result=gate.command({0.2,0,0,0.3},true,ros,wall,true);
  assert(result.stop && neutral(result.command)); // L2 cancels; never reverse.
  arm(); result=gate.command(raw,true,ros,wall,true);
  assert(result.stop); // Both triggers: detect raw L2 even if teleop cancels it.
  arm(); assert(!gate.allowed(ros+0.4,wall+0.4,true));
  gate.observe(AUTO,ros+0.4,ros+0.4,wall+0.4,true,true);
  assert(!gate.active()); // Mode freshness/recovery require a new STOP.
  arm(); assert(!gate.allowed(ros,wall,false)); // Joy/ready/deadman loss latches off.
  arm(); assert(!gate.allowed(ros-0.01,wall,true)); // Even a small ROS time reversal.
  arm(); assert(!gate.allowed(ros,wall+0.31,true)); // Receipt stale with paused ROS clock.
  arm(); result=gate.command({0,std::numeric_limits<double>::quiet_NaN(),0,0},false,ros,wall,true);
  assert(result.stop && neutral(result.command));
  arm(); gate.trip(); assert(!gate.active()); // Explicit STOP immediately removes authority.
  gate.observe(MANUAL,ros,ros,wall,true,true);
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.throttle==raw.throttle); // MANUAL auxiliary teleop stays raw.
  gate.observe(AUTO,ros,ros,wall,true,true);
  assert(!gate.active()); // A tripped gate cannot rearm by switching modes.
  arm();
  gate.observe(MANUAL,ros,ros,wall,true,false);
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.throttle==raw.throttle);
  ros+=5; wall+=5; // Manual driving can last longer than the AUTO watchdog.
  gate.observe(MANUAL,ros,ros,wall,true,false);
  gate.observe(AUTO,ros,ros,wall,true,false);
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.steering==raw.steering && result.command.throttle==0.1);
  gate.throttle=0; arm();
  result=gate.command(raw,false,ros,wall,true);
  assert(!result.stop && result.command.throttle==0 && result.command.steering==raw.steering);

  ManualFixedThrottle nonneutral;
  nonneutral.throttle=0.1;
  nonneutral.observe(STOP,10,10,1,true,true);
  nonneutral.observe(AUTO,10.1,10.1,1.1,true,false);
  assert(!nonneutral.active()); // External AUTO request with axes held cannot arm.
  TeleopInput input;
  std::vector<float> axes(8,0);
  assert(input.can_arm(axes,false)); // Arming uses raw neutral, not injected throttle.
  axes[5]=0.2; assert(!input.can_arm(axes,false));

  {
    ManualFixedThrottle id;
    id.r2_gate=true;
    assert(!id.valid_settings(0.3)); // R2 gate never enables an unspecified fixed value.
    id.throttle=0.137;
    assert(id.valid_settings(0.3));
    double time=100,steady=10;
    auto command=[&](double r2,bool healthy=true) {
      return id.command({0.4,r2>=0 && r2<=1 ? r2*.3 : 0,0,0},false,
                        time,steady,healthy,r2);
    };
    auto tick=[&](double r2) {
      time+=0.05; steady+=0.05;
      id.observe(AUTO,time,time,steady,true,r2<=0.01);
      return command(r2);
    };
    auto start=[&](double held_r2) {
      time+=1; steady+=1;
      id.observe(STOP,time,time,steady,true,true);
      time+=0.01; steady+=0.01;
      id.observe(AUTO,time,time,steady,true,true);
      const auto entry=command(held_r2);
      assert(!entry.stop && neutral(entry.command));
      assert(std::string(id.phase())=="entry_neutral");
      for(int i=0;i<8;++i) {tick(held_r2);}
    };
    // A node started in AUTO cannot engage even with a valid R2 press.
    id.observe(AUTO,time,time,steady,true,true);
    auto out=command(1.0);
    assert(out.stop && neutral(out.command) && std::string(id.phase())=="fault");

    start(0.8); // Pressed during entry cannot automatically engage when .3 s elapses.
    assert(id.active() && id.r2_pressed() && id.effective_throttle()==0);
    assert(std::string(id.phase())=="coast");
    out=tick(0.08);
    assert(out.command.throttle==0); // A partly released, still-held trigger cannot arm.
    out=tick(ManualFixedThrottle::R2_RELEASE);
    assert(!out.stop && out.command.steering==0.4 && out.command.throttle==0);
    assert(out.command.brake==0 && out.command.reverse==0 && !id.r2_pressed());
    out=tick(0.08);
    assert(out.command.throttle==0); // Below the press threshold stays coasting.
    out=tick(ManualFixedThrottle::R2_PRESS);
    assert(!out.stop && out.command.throttle==0.137 && out.command.steering==0.4);
    assert(std::string(id.phase())=="drive" && id.effective_throttle()==0.137);
    const auto drive_sequence=id.phase_sequence();
    for(double depth:{0.2,0.7,1.0,0.07}) {
      out=tick(depth);
      assert(out.command.throttle==0.137 && id.phase_sequence()==drive_sequence);
    }
    out=tick(ManualFixedThrottle::R2_RELEASE);
    assert(!out.stop && out.command.throttle==0 && out.command.brake==0 && out.command.reverse==0);
    assert(out.command.steering==0.4 && id.active() && id.phase_sequence()==drive_sequence+1);

    // A release seen only during the entry-neutral interval is insufficient.
    time+=1; steady+=1;
    id.observe(STOP,time,time,steady,true,true);
    time+=0.01; steady+=0.01;
    id.observe(AUTO,time,time,steady,true,true);
    command(0);
    tick(0); tick(0);
    for(int i=0;i<7;++i) {out=tick(0.8);}
    assert(!out.stop && out.command.throttle==0 && std::string(id.phase())=="coast");
    tick(0); out=tick(0.8);
    assert(out.command.throttle==0.137);

    tick(1.0);
    assert(!id.allowed(time,steady,false));
    assert(id.effective_throttle()==0 && std::string(id.phase())=="fault");
    out=tick(1.0);
    assert(out.stop && neutral(out.command)); // Healthy AUTO heartbeat/input cannot restart.
    start(0);
    out=tick(0.7);
    assert(out.command.throttle==0.137);
    out=id.command({0.4,0,0,.2},true,time,steady,true,0.7);
    assert(out.stop && neutral(out.command) && id.effective_throttle()==0);
    start(0);
    out=id.command({0.4,0,1,0},false,time,steady,true,0.7);
    assert(out.stop && out.command.brake==1 && out.command.throttle==0);
    assert(id.effective_throttle()==0);

    for(double bad:{-0.1,1.1,std::numeric_limits<double>::quiet_NaN()}) {
      start(0);
      out=command(bad);
      assert(out.stop && neutral(out.command));
    }
    start(0);
    tick(1.0);
    assert(!id.allowed(time-0.01,steady,true));
    assert(std::string(id.phase())=="fault" && id.effective_throttle()==0);
    start(0);
    tick(1.0);
    assert(!id.allowed(time,steady+0.31,true)); // Paused ROS clock cannot hide a wall timeout.
    assert(std::string(id.phase())=="fault");

    start(0);
    tick(1.0);
    id.observe(MANUAL,time,time,steady,true,false);
    out=command(0.9);
    assert(!out.stop && std::abs(out.command.throttle-0.27)<1e-12 && std::string(id.phase())=="inactive");
    time+=5; steady+=5;
    id.observe(MANUAL,time,time,steady,true,false);
    id.observe(AUTO,time,time,steady,true,false);
    out=command(0.9);
    assert(!out.stop && neutral(out.command)); // New entry even on healthy MANUAL->AUTO.
    for(int i=0;i<8;++i) {out=tick(0.9);}
    assert(!out.stop && out.command.throttle==0 && id.active());
    tick(0); out=tick(0.9);
    assert(out.command.throttle==0.137);

    id.trip();
    assert(std::string(id.phase())=="fault" && id.effective_throttle()==0);
    id.observe(MANUAL,time,time,steady,true,true);
    id.observe(AUTO,time,time,steady,true,true);
    out=command(0.9);
    assert(out.stop && neutral(out.command)); // Mode handover is not a substitute for STOP.
    id.observe(STOP,time,time,steady,false,true);
    const auto fault_sequence=id.phase_sequence();
    id.observe(STOP,time,time,steady,false,true);
    assert(std::string(id.phase())=="fault" && id.phase_sequence()==fault_sequence);
    id.observe(STOP,time,time,steady,true,true);
    assert(std::string(id.phase())=="inactive");
  }

  std::cout<<"manual fixed throttle: passed\n";
}
