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
// Keep explicit 0.2 s coverage for configurable long dwell independently of defaults.
EscSequenceSettings enabled() {EscSequenceSettings s;s.enabled=true;s.reverse_neutral_s=.2;return s;}
void dwell_reverse(EscSequencer &s,double t) {
  s.apply(frame(),t);
  assert(s.apply(frame(0,200),t+.01).reverse_milli==0);
  for(int i=2;i<=20;++i)assert(s.apply(frame(0,200),t+i*.01).reverse_milli==0);
  assert(s.apply(frame(0,200),t+.22).reverse_milli==200);
}
int main() {
  // New default is a short neutral sample; it must still be emitted before reverse.
  EscSequenceSettings defaults;assert(defaults.enabled);
  assert(defaults.reverse_neutral_s==.01);
  EscSequencer short_dwell(defaults);
  short_dwell.apply(frame(),20);
  assert(short_dwell.apply(frame(0,200),20.01).reverse_milli==0);
  assert(short_dwell.apply(frame(0,200),20.015).reverse_milli==0);
  assert(short_dwell.apply(frame(0,200),20.0201).reverse_milli==200);
  // Full forward -> brake -> new short neutral -> reverse using default settings.
  short_dwell.reset();short_dwell.apply(frame(),21);short_dwell.apply(frame(300),21.01);
  for(int i=0;i<20;++i)assert(short_dwell.apply(frame(0,200),21.02+i*.01).brake_milli==1000);
  assert(short_dwell.apply(frame(0,200),21.221).reverse_milli==0);
  assert(short_dwell.apply(frame(0,200),21.226).reverse_milli==0);
  assert(short_dwell.apply(frame(0,200),21.2311).reverse_milli==200);
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
  // Forward -> reverse demand first brakes, then holds neutral, then reverses.
  assert(s.apply(frame(250),2.30).throttle_milli==250);
  for(int i=0;i<=19;++i) {
    f=s.apply(frame(0,i%2 ? 700 : 180),2.31+i*.01);
    assert(f.brake_milli==1000 && f.reverse_milli==0 && f.throttle_milli==0);
    assert(std::string(s.name())=="reverse_braking");
  }
  for(int i=0;i<=19;++i) {
    f=s.apply(frame(0,180),2.52+i*.01);
    assert(f.brake_milli==0 && f.reverse_milli==0 && f.throttle_milli==0);
  }
  assert(s.apply(frame(0,180),2.73).reverse_milli==180);
  // Neutral cancels entry but preserves the positive pulse's pending-first-brake latch.
  s.apply(frame(0,0,500),2.74);
  f=s.apply(frame(),2.75);assert(f.throttle_milli==0 && f.brake_milli==0);
  f=s.apply(frame(0,0,500),2.76);assert(f.throttle_milli==0 && f.brake_milli==500);
  assert(std::string(s.name())=="braking");assert(!s.esc().brake_rejected());
  // Changing from the positive pulse to reverse also brakes before neutral/backing.
  s.reset();dwell_reverse(s,3);s.apply(frame(0,0,500),3.23);
  for(int i=0;i<=19;++i) {
    f=s.apply(frame(0,100),3.24+i*.01);
    assert(f.throttle_milli==0 && f.reverse_milli==0 && f.brake_milli==1000);
  }
  for(int i=0;i<=19;++i)assert(s.apply(frame(0,100),3.45+i*.01).reverse_milli==0);
  assert(s.apply(frame(0,100),3.66).reverse_milli==100);
  // Output clock gaps abort and latch until reset. No automatic pulse resume.
  s.apply(frame(0,0,500),3.67);
  f=s.apply(frame(0,0,500),3.9);
  assert(s.failed() && f.flags==COMMAND_VALID && f.throttle_milli==0);
  assert(s.apply(frame(100),3.91).flags==COMMAND_VALID);
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
  // Real trigger handover can spend any length in neutral before L2.
  for(int neutral_ticks : {1,3,37,1000}) {
    EscSequenceSettings cfg;EscSequencer history(cfg);
    double t=30;
    history.apply(frame(),t);t+=.01;history.apply(frame(200),t);
    for(int i=0;i<neutral_ticks;++i) {
      t+=.01;history.apply(frame(),t);
      assert(history.esc().negative_is_brake());
    }
    t+=.01;f=history.apply(frame(0,250),t);
    assert(f.brake_milli==1000 && f.reverse_milli==0);
    assert(std::string(history.name())=="reverse_braking");
    // Braking -> neutral -> reverse must finish without restarting the brake latch.
    bool saw_neutral=false,saw_reverse=false;
    for(int i=0;i<30;++i) {
      t+=.01;f=history.apply(frame(0,250),t);
      if(f.reverse_milli) {assert(saw_neutral && f.reverse_milli==250);saw_reverse=true;}
      else if(!f.brake_milli) {assert(!saw_reverse);saw_neutral=true;}
      else {assert(!saw_neutral && !saw_reverse && f.brake_milli==1000);}
    }
    assert(saw_reverse);
    // STOP/HOST loss erases the otherwise persistent history.
    auto lost=frame();lost.flags=COMMAND_VALID;history.apply(lost,t+.01);
    assert(std::string(history.esc().next_negative_name())=="unknown");
  }
  // Dedicated brake after forward + neutral also consumes the first negative latch.
  s.reset();s.apply(frame(),50);s.apply(frame(200),50.01);s.apply(frame(),50.02);
  assert(s.apply(frame(0,0,500),50.03).brake_milli==500);
  s.apply(frame(),50.04);
  assert(!s.esc().negative_is_brake());
  assert(s.apply(frame(0,200),50.05).reverse_milli==0);
  assert(std::string(s.name())=="reverse_neutral");
  // Disabled profile keeps legacy behavior, including rejection from reverse.
  EscSequenceSettings disabled_settings;disabled_settings.enabled=false;
  EscSequencer legacy(disabled_settings);legacy.apply(frame(),0);legacy.apply(frame(0,200),.01);
  f=legacy.apply(frame(0,0,400),.02);assert(f.brake_milli==0 && f.throttle_milli==0);
  // Cancelling reverse during the brake stage never finishes backing on its own.
  for(int choice=0;choice<4;++choice) {
    s.reset();s.apply(frame(),7);s.apply(frame(200),7.01);
    assert(s.apply(frame(0,200),7.02).brake_milli==1000);
    // Varying reverse magnitude must not restart the fixed brake dwell.
    assert(s.apply(frame(0,700),7.04).brake_milli==1000);
    if(choice==0) {f=s.apply(frame(),7.05);assert(f.brake_milli==0 && f.reverse_milli==0);}
    if(choice==1) {f=s.apply(frame(300),7.05);assert(f.throttle_milli==300 && f.reverse_milli==0);}
    if(choice==2) {
      f=s.apply(frame(0,0,500),7.05);assert(f.brake_milli==500);
      for(int i=0;i<50;++i)assert(s.apply(frame(0,0,500),7.06+i*.01).reverse_milli==0);
    }
    if(choice==3) {s.apply(disarmed,7.05);assert(s.esc().state()==EscState::State::unknown);}
  }
  s.reset();s.apply(frame(),8);s.apply(frame(200),8.01);s.apply(frame(0,200),8.02);
  assert(s.apply(frame(0,200),8.3).flags==COMMAND_VALID && s.failed());
  // Settings are bounded and finite, including values lost to PWM quantization.
  for(int i=0;i<7;++i) {
    auto cfg=enabled();
    if(i==0)cfg.brake_entry_throttle=.001;
    if(i==1)cfg.brake_entry_s=0;
    if(i==2)cfg.reverse_neutral_s=3;
    if(i==3)cfg.max_step_gap_s=.3;
    if(i==4)cfg.brake_entry_s=std::numeric_limits<double>::quiet_NaN();
    if(i==5)cfg.reverse_brake_command=.001;
    if(i==6)cfg.reverse_brake_s=0;
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
  // Real MANUAL mapping: held L2 while forward -> brake -> neutral -> reverse.
  session.start(12.5);session.output({},true,12.51,1,0);session.status(status);
  axes[5]=.8F;axes[4]=0;
  assert(session.output(joy.command(axes,false),true,12.52,1,0,{},true).throttle_milli==240);
  axes[5]=0;axes[4]=.8F;
  bool saw_brake=false,saw_neutral=false,saw_reverse=false;
  for(int i=0;i<48;++i) {
    f=session.output(joy.command(axes,false),true,12.53+i*.01,1,0,{},true);
    assert(f.throttle_milli==0);
    if(f.brake_milli) {assert(!saw_neutral && !saw_reverse && f.brake_milli==1000);saw_brake=true;}
    else if(f.reverse_milli) {assert(saw_brake && saw_neutral && f.reverse_milli==240);saw_reverse=true;}
    else {assert(saw_brake && !saw_reverse);saw_neutral=true;}
  }
  assert(saw_brake && saw_neutral && saw_reverse);
  session.start(13);session.output({},true,13.01,1,0);session.status(status);
  session.output({0,.2,0,0},true,13.3,1,0);
  assert(session.phase()==HostSession::Phase::disarmed);
  assert(session.sequencer().failed());
  std::cout << "ESC sequencer: pulse/dwell, repeated demand, cancellation, timing faults and HostSession interlocks passed\n";
}
