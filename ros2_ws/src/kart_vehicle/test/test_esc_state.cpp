#include "kart_vehicle/esc_state.hpp"
#include "kart_vehicle/host_session.hpp"
#include <cassert>
#include <iostream>
#include <string>
using namespace kart_vehicle;
using State = EscState::State;
CommandFrame input(int forward=0,int reverse=0,int brake=0) {
  CommandFrame f; f.flags=COMMAND_VALID|HOST_REQUEST;
  f.throttle_milli=forward; f.reverse_milli=reverse; f.brake_milli=brake;
  return f;
}
EscState at(State state) {
  EscState s;
  if(state==State::unknown)return s;
  s.apply(input());
  if(state==State::forward || state==State::neutral_after_forward || state==State::brake || state==State::neutral_after_brake) {
    s.apply(input(300));
  }
  if(state==State::brake || state==State::neutral_after_brake || state==State::reverse || state==State::neutral_after_reverse) {
    s.apply(input(0,300));
  }
  if(state==State::neutral_after_forward || state==State::neutral_after_brake || state==State::neutral_after_reverse)s.apply(input());
  assert(s.state()==state);return s;
}
int main() {
  // Every explicit neutral-history state x neutral / positive / negative / brake.
  const State states[]={State::unknown,State::neutral_initial,State::forward,State::neutral_after_forward,
                        State::brake,State::neutral_after_brake,State::reverse,State::neutral_after_reverse};
  const CommandFrame commands[]={input(),input(300),input(0,300),input(0,0,700)};
  const State expected[][4]={
    {State::neutral_initial,State::unknown,State::unknown,State::unknown},
    {State::neutral_initial,State::forward,State::reverse,State::neutral_initial},
    {State::neutral_after_forward,State::forward,State::brake,State::brake},
    {State::neutral_after_forward,State::forward,State::brake,State::brake},
    {State::neutral_after_brake,State::forward,State::brake,State::brake},
    {State::neutral_after_brake,State::forward,State::reverse,State::neutral_after_brake},
    {State::neutral_after_reverse,State::forward,State::reverse,State::neutral_after_reverse},
    {State::neutral_after_reverse,State::forward,State::reverse,State::neutral_after_reverse}};
  for(int row=0;row<8;++row) {
    for(int col=0;col<4;++col) {
      auto s=at(states[row]);auto f=s.apply(commands[col]);
      assert(s.state()==expected[row][col]);
      assert(s.brake_rejected()==(col==3 && (row==1 || row==5 || row==6 || row==7)));
      if(s.is_neutral() || s.state()==State::unknown) {
        assert(f.throttle_milli==0 && f.reverse_milli==0 && f.brake_milli==0);
      } else if(s.state()==State::forward) {assert(f.throttle_milli==300 && f.reverse_milli==0 && f.brake_milli==0);}
      else if(s.state()==State::reverse) {assert(f.reverse_milli==300 && f.throttle_milli==0 && f.brake_milli==0);}
      else {assert(f.brake_milli==(col==3 ? 700 : 300) && f.reverse_milli==0 && f.throttle_milli==0);}
    }
  }
  auto s=at(State::forward);
  for (int i=0;i<1000;++i) {
    const auto f=s.apply(input(0,500));
    assert(s.state()==State::brake && f.brake_milli==500 && f.reverse_milli==0);
  }
  s.apply(input()); s.apply(input(0,300)); assert(s.state()==State::reverse);
  s.apply(input(300)); assert(s.state()==State::forward); // Not a braking phase.
  CommandFrame disarmed; disarmed.flags=COMMAND_VALID;
  s.apply(disarmed); assert(s.state()==State::unknown);
  // Explicit brake from neutral must not drive backwards, even when held.
  s.apply(input());
  for (int i=0;i<1000;++i) {assert(s.apply(input(0,0,1000)).brake_milli==0);}
  // Forward -> arbitrarily long neutral -> first negative is still brake.
  for(bool dedicated : {false,true}) {
    auto history=at(State::forward);
    for(int i=0;i<1000;++i) {
      history.apply(input());
      assert(history.state()==State::neutral_after_forward && history.negative_is_brake());
    }
    auto brake=history.apply(dedicated ? input(0,0,250) : input(0,250));
    assert(brake.brake_milli==250 && brake.reverse_milli==0);
    assert(history.state()==State::brake);
    // Holding negative never transitions to reverse.
    for(int i=0;i<100;++i)assert(history.apply(input(0,250)).brake_milli==250);
    history.apply(input());
    assert(!history.negative_is_brake());
    assert(history.apply(input(0,250)).reverse_milli==250);
    history.apply(input(200));history.apply(input());
    assert(history.negative_is_brake()); // A new positive drive primes braking again.
    history.reset();assert(std::string(history.next_negative_name())=="unknown");
    assert(!history.negative_is_brake());
  }
  // Quantization and HOST loss must not leave a false forward/brake history.
  HostSession session; session.start(0);
  session.output({},true,0.01,-1,0);
  StatusFrame status; status.selector=RcSelector::automatic; status.active_path=ActivePath::automatic;
  session.status(status);
  session.output({0,0.0001,0,0},true,0.02,-1,0);
  assert(session.esc().state()==State::neutral_initial);
  // One milli still produces exactly neutral PWM with firmware's integer /2.
  assert(session.output({0,0.001,0,0},true,0.025,-1,0).throttle_milli==0);
  assert(session.esc().state()==State::neutral_initial);
  auto f=session.output({0,0,1,0},true,0.03,-1,0);
  assert(f.brake_milli==0 && session.esc().brake_rejected());
  session.output({0,0.5,0,0},true,0.04,-1,0);
  assert(session.esc().state()==State::forward);
  status.active_path=ActivePath::manual; session.status(status);
  assert(session.esc().state()==State::unknown);
  assert(session.output({0,0,1,0},true,0.05,-1,0).flags==COMMAND_VALID);
  s=at(State::neutral_initial);
  assert(s.apply(input(2)).throttle_milli==2 && s.state()==State::forward);
  assert(s.apply(input(0,1)).reverse_milli==0 && s.state()==State::neutral_after_forward);
  std::cout << "ESC command model: 32 transitions, held brake, reset, quantization passed\n";
}
