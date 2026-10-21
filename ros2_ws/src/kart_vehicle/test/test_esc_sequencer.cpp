#include "kart_vehicle/host_session.hpp"
#include "kart_system/joy_controls.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace kart_vehicle;
CommandFrame frame(int forward=0,int reverse=0,int brake=0) {
  CommandFrame f;f.flags=COMMAND_VALID|HOST_REQUEST;f.steering_milli=321;
  f.throttle_milli=forward;f.reverse_milli=reverse;f.brake_milli=brake;return f;
}
EscSequenceSettings enabled() {EscSequenceSettings s;s.enabled=true;return s;}
void dwell_reverse(EscSequencer &s,double t) {
  s.apply(frame(),t);
  assert(s.apply(frame(0,200),t+.01).reverse_milli==0);
  for(int i=2;i<=20;++i)assert(s.apply(frame(0,200),t+i*.01).reverse_milli==0);
  assert(s.apply(frame(0,200),t+.22).reverse_milli==200);
}
int main() {
  EscSequencer s(enabled());dwell_reverse(s,1);
  auto f=s.apply(frame(0,0,400),1.23);
  assert(f.throttle_milli==80 && f.brake_milli==0 && f.reverse_milli==0);
  assert(f.steering_milli==321 && s.esc().state()==EscState::State::forward);
  // Repeated brake requests and changing amount do not restart the pulse.
  assert(s.apply(frame(0,0,500),1.25).throttle_milli==80);
  f=s.apply(frame(0,0,500),1.29);
  assert(f.brake_milli==500 && f.throttle_milli==0);
  for(int i=0;i<100;++i) {
    f=s.apply(frame(0,0,300),1.30+i*.01);
    assert(f.brake_milli==300 && f.throttle_milli==0 && f.reverse_milli==0);
  }
  assert(s.esc().state()==EscState::State::brake);
  // Forward demand from braking takes effect directly; reverse gets neutral dwell.
  assert(s.apply(frame(250),2.30).throttle_milli==250);
  assert(s.apply(frame(0,180),2.31).reverse_milli==0);
  for(int i=1;i<=19;++i)assert(s.apply(frame(0,180),2.31+i*.01).reverse_milli==0);
  assert(s.apply(frame(0,180),2.52).reverse_milli==180);
  // Neutral cancels entry; a later brake request cannot restart a positive pulse.
  s.apply(frame(0,0,500),2.53);
  f=s.apply(frame(),2.54);assert(f.throttle_milli==0 && f.brake_milli==0);
  f=s.apply(frame(0,0,500),2.55);assert(f.throttle_milli==0 && f.brake_milli==0);
  assert(std::string(s.name())=="rejected");
  assert(s.esc().brake_rejected());
  // Demand changes to reverse during the positive pulse insert neutral before backing.
  s.reset();dwell_reverse(s,3);
  s.apply(frame(0,0,500),3.23);
  assert(s.apply(frame(0,100),3.24).throttle_milli==0);
  for(int i=1;i<=19;++i)assert(s.apply(frame(0,100),3.24+i*.01).reverse_milli==0);
  assert(s.apply(frame(0,100),3.45).reverse_milli==100);
  // Output clock gaps abort and latch until reset. No automatic pulse resume.
  s.apply(frame(0,0,500),3.46);
  f=s.apply(frame(0,0,500),3.7);
  assert(s.failed() && f.flags==COMMAND_VALID && f.throttle_milli==0);
  assert(s.apply(frame(100),3.71).flags==COMMAND_VALID);
  s.reset();s.apply(frame(),4);assert(!s.failed());
  s.apply(frame(100),3.9);assert(s.failed());
  s.reset();s.apply(frame(),std::numeric_limits<double>::quiet_NaN());assert(s.failed());
  // HOST withdrawal cancels the pulse and forgets command history.
  s.reset();dwell_reverse(s,5);s.apply(frame(0,0,500),5.23);
  auto disarmed=frame();disarmed.flags=COMMAND_VALID;
  s.apply(disarmed,5.24);assert(s.esc().state()==EscState::State::unknown);
  assert(std::string(s.name())=="idle");
  // Quantized neutral cannot generate a pulse or directional transition.
  s.reset();s.apply(frame(),6);f=s.apply(frame(0,0,1),6.01);
  assert(f.brake_milli==0 && f.throttle_milli==0);
  // Disabled profile keeps legacy behavior, including rejection from reverse.
  EscSequencer legacy;legacy.apply(frame(),0);legacy.apply(frame(0,200),.01);
  f=legacy.apply(frame(0,0,400),.02);assert(f.brake_milli==0 && f.throttle_milli==0);
  // Settings are bounded and finite, including values lost to PWM quantization.
  for(int i=0;i<5;++i) {
    auto cfg=enabled();
    if(i==0)cfg.brake_entry_throttle=.001;
    if(i==1)cfg.brake_entry_s=0;
    if(i==2)cfg.reverse_neutral_s=3;
    if(i==3)cfg.max_step_gap_s=.3;
    if(i==4)cfg.brake_entry_s=std::numeric_limits<double>::quiet_NaN();
    bool rejected=false;try{EscSequencer invalid(cfg);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  // Real HostSession path: fresh semantic brake requests -> quantized wire frames.
  HostSession session;session.configure_esc(enabled());session.start(10);
  session.output({},true,10.01,-1,0);
  StatusFrame status;status.selector=RcSelector::automatic;status.active_path=ActivePath::automatic;
  session.status(status);
  for(int i=0;i<23;++i)f=session.output({.2,0,0,.2},true,10.02+i*.01,-1,0);
  assert(f.reverse_milli==200);
  f=session.output({.2,0,.4,0},true,10.25,-1,0);assert(f.throttle_milli==80 && f.steering_milli==-200);
  assert(session.output({.2,0,.4,0},false,10.26,-1,0).flags==COMMAND_VALID);
  assert(session.phase()==HostSession::Phase::disarmed);
  assert(session.output({0,0,.4,0},true,10.27,-1,0).flags==COMMAND_VALID);
  // Board authority loss and STOP interrupt immediately, even mid-pulse.
  for(bool stop : {true,false}) {
    session.start(11);session.output({},true,11.01,1,0);session.status(status);
    for(int i=0;i<23;++i)session.output({0,0,0,.2},true,11.02+i*.01,1,0);
    assert(session.output({0,0,.4,0},true,11.25,1,0).throttle_milli==80);
    if(stop)session.stop();else{auto lost=status;lost.active_path=ActivePath::manual;session.status(lost);}
    assert(session.output({0,0,.4,0},true,11.26,1,0).flags==COMMAND_VALID);
    assert(session.esc().state()==EscState::State::unknown);
  }
  session.start(11.5);session.output({},true,11.51,1,0);session.status(status);
  for(int i=0;i<23;++i)session.output({0,0,0,.2},true,11.52+i*.01,1,0);
  assert(session.output({0,0,.4,0},true,11.75,1,0).throttle_milli==80);
  assert(session.output({0,.2,.4,0},true,11.76,1,0).flags==COMMAND_VALID);
  assert(session.phase()==HostSession::Phase::disarmed); // Invalid exclusive demand cancels pulse.
  session.start(11.8);session.output({},true,11.81,1,0);
  session.output({},true,11.95,1,0);
  assert(session.phase()==HostSession::Phase::disarmed); // Gap during handshake also aborts.
  kart_system::TeleopInput joy;
  std::vector<float> axes(6,0);axes[4]=.8F;axes[5]=.6F;
  const auto joy_brake=joy.command(axes,true);
  assert(joy_brake.brake==1 && joy_brake.throttle==0 && joy_brake.reverse==0);
  session.start(12);session.output({},true,12.01,1,0);session.status(status);
  for(int i=0;i<23;++i)session.output({0,0,0,.2},true,12.02+i*.01,1,0);
  assert(session.output(joy_brake,true,12.25,1,0,{},true).throttle_milli==80);
  for(int i=1;i<=7;++i)f=session.output(joy_brake,true,12.25+i*.01,1,0,{},true);
  assert(f.brake_milli==1000 && f.throttle_milli==0 && f.reverse_milli==0);
  axes[4]=axes[5]=0;
  f=session.output(joy.command(axes,false),true,12.33,1,0,{},true);
  assert(f.brake_milli==0 && f.throttle_milli==0 && f.reverse_milli==0);
  session.start(13);session.output({},true,13.01,1,0);session.status(status);
  session.output({0,.2,0,0},true,13.3,1,0);
  assert(session.phase()==HostSession::Phase::disarmed);
  assert(session.sequencer().failed());
  std::cout << "ESC sequencer: pulse/dwell, repeated demand, cancellation, timing faults and HostSession interlocks passed\n";
}
