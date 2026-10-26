#include "kart_system/joy_controls.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace kart_system;
int main() {
  TeleopInput input;
  std::vector<float> axes(8,0);
  assert(input.valid_axes(axes) && input.can_arm(axes,false));
  axes[0]=-1; axes[5]=1;
  auto command=input.command(axes,false);
  assert(command.steering==1 && command.throttle==0.3 && command.reverse==0);
  assert(!input.can_arm(axes,false));
  axes[0]=0; axes[4]=1;
  assert(neutral(input.command(axes,false)) && !input.can_arm(axes,false)); // Pressed triggers cancel, not safe to arm.
  axes[5]=0; command=input.command(axes,false);
  assert(command.reverse==0.3 && command.throttle==0);
  command=input.command(axes,true); assert(command.brake==1 && command.reverse==0);
  axes[4]=-1; assert(!input.valid_axes(axes));
  axes[4]=std::numeric_limits<float>::quiet_NaN(); assert(!input.valid_axes(axes));
  assert(!input.valid_axes({}));

  JoyButtons unguarded;
  std::vector<int32_t> unguarded_keys(17,0);
  auto unguarded_actions=unguarded.read(unguarded_keys);
  assert(unguarded.deadman_button==-1 && unguarded_actions->deadman);
  unguarded_keys[2]=1; unguarded_actions=unguarded.read(unguarded_keys);
  assert(unguarded_actions->manual && unguarded_actions->deadman); // Triangle alone.
  unguarded_keys[2]=0; unguarded_actions=unguarded.read(unguarded_keys);
  assert(unguarded_actions->deadman && !unguarded_actions->manual_held); // Mode hold handles release below.
  unguarded_keys[0]=1; unguarded_actions=unguarded.read(unguarded_keys);
  assert(unguarded_actions->stop); // Explicit STOP remains available.
  unguarded.reset(); unguarded_keys[2]=1;
  assert(!unguarded.read(unguarded_keys)->manual); // Held at reconnect must not rearm.

  JoyButtons recording_buttons;
  std::vector<int32_t> recording_keys(17,0);
  recording_buttons.read(recording_keys);
  recording_keys[4]=1;
  auto recording_actions=recording_buttons.read(recording_keys);
  assert(recording_actions->bag_start && !recording_actions->bag_stop);
  assert(!recording_buttons.read(recording_keys)->bag_start); // Hold is not repeated START.
  recording_keys[4]=0; recording_buttons.read(recording_keys);
  recording_keys[4]=recording_keys[5]=1;
  recording_actions=recording_buttons.read(recording_keys);
  assert(recording_actions->bag_stop && !recording_actions->bag_start); // STOP wins.
  recording_buttons.reset(); recording_actions=recording_buttons.read(recording_keys);
  assert(!recording_actions->bag_start && !recording_actions->bag_stop); // No reconnect side effect.
  recording_buttons.bag_start_button=recording_buttons.bag_stop_button=-1;
  recording_keys.assign(17,0); recording_buttons.read(recording_keys);
  recording_keys[4]=recording_keys[5]=1;
  recording_actions=recording_buttons.read(recording_keys);
  assert(!recording_actions->bag_start && !recording_actions->bag_stop);

  JoyButtons buttons;
  buttons.deadman_button=4; // Optional L1 deadman remains supported.
  std::vector<int32_t> keys(17,0);
  keys[4]=1; keys[2]=1; keys[15]=1;
  auto actions=buttons.read(keys);
  assert(actions && actions->deadman && !actions->manual && actions->steering_delta==0);
  keys[2]=keys[15]=0; buttons.read(keys);
  keys[2]=keys[15]=1; actions=buttons.read(keys);
  assert(actions->manual && !actions->automatic && actions->steering_delta==0.001);
  actions=buttons.read(keys); assert(!actions->manual && actions->steering_delta==0); // No repeats when held.
  buttons.reset(); actions=buttons.read(keys);
  assert(!actions->manual && actions->steering_delta==0); // Reconnect with controls held.
  keys.assign(17,0); buttons.read(keys);
  keys[3]=keys[13]=keys[16]=1; actions=buttons.read(keys);
  assert(actions->automatic && actions->throttle_delta==0.001 && actions->steering_delta==-0.001);
  keys.assign(17,0); buttons.read(keys);
  keys[0]=keys[1]=keys[14]=1; actions=buttons.read(keys);
  assert(actions->stop && actions->brake && !actions->deadman && actions->throttle_delta==-0.001);
  assert(!buttons.read({}));
  actions=buttons.read(keys); assert(!actions->stop && actions->throttle_delta==0);
  buttons.auto_button=-1; keys.assign(17,0); buttons.read(keys);
  keys[3]=1; actions=buttons.read(keys); assert(!actions->automatic);
  {
    JoyButtons held_buttons;
    JoyModeHold hold;
    std::vector<int32_t> held(17,0);
    auto step=[&]() {return hold.update(*held_buttons.read(held));};
    held[2]=1;
    assert(step()==0 && !hold.active()); // Held at startup cannot arm.
    held[2]=0; assert(step()==0);
    held[2]=1; assert(step()==MANUAL && hold.active());
    for (int n=0;n<100;++n) assert(step()==0); // No repeated mode request.
    held[2]=0; assert(step()==STOP && !hold.active()); // Also before mode acknowledgement.
    held[3]=1; assert(step()==AUTO);
    held[3]=0; held[2]=1; assert(step()==MANUAL); // Direct handover in one sample.
    held[2]=0; held[3]=1; assert(step()==AUTO);
    held[2]=1; assert(step()==STOP); // Conflicting buttons fail closed.
    held[3]=0; assert(step()==0 && !hold.active()); // No fallback to remaining button.
    held[2]=0; assert(step()==0);
    held[2]=1; assert(step()==MANUAL);
    held[0]=1; assert(step()==STOP);
    held[0]=0; assert(step()==0 && !hold.active()); // STOP release never resumes.
    held.assign(17,0); step(); held[3]=1; assert(step()==AUTO);
    hold.reset(); // External STOP / fault, while square remains held.
    for (int n=0;n<100;++n) assert(step()==0 && !hold.active());
    held[3]=0; step(); held[3]=1; assert(step()==AUTO);
    hold.reset(); held_buttons.reset(); // Input loss and reconnect with button held.
    assert(step()==0 && !hold.active());
    held[3]=0; step(); held[3]=1; assert(step()==AUTO);
    held[3]=0; assert(step()==STOP);
    held_buttons.auto_button=-1;
    step(); held[3]=1; assert(step()==0); // Disabled AUTO has no hold permission.
    held[2]=1; assert(step()==MANUAL);
  }
  {
    JoyBrakePulse pulse;
    pulse.duration_s=.05; // Longer pulse remains configurable.
    const Command brake{.2,0,1,0};
    // Startup/reconnection with circle held must never synthesize propulsion.
    assert(pulse.command(brake,true,true,10).brake==1);
    pulse.command({},false,true,10.01);
    auto out=pulse.command(brake,true,true,10.02);
    assert(out.throttle==.08 && out.brake==0 && out.reverse==0 && out.steering==.2);
    assert(pulse.command(brake,true,true,10.06).throttle==.08);
    assert(pulse.command(brake,true,true,10.08).brake==1);
    for(int n=0;n<100;++n) {assert(pulse.command(brake,true,true,10.09+.01*n).brake==1);}
    // Release cancels immediately, and a new press starts exactly one pulse.
    const Command reverse{-.3,0,0,.2};
    assert(pulse.command(reverse,false,true,11.09).reverse==.2);
    assert(pulse.command(brake,true,true,11.10).throttle==.08);
    assert(pulse.command({},false,true,11.11).throttle==0);
    pulse.command(brake,true,true,11.12);
    pulse.reset(); // STOP, mode transition, input failure or deadman release.
    assert(pulse.command(brake,true,true,11.13).brake==1);
    pulse.command({},false,true,11.14);
    pulse.command(brake,true,true,11.15);
    assert(pulse.command(brake,true,false,11.16).brake==1); // AUTO is unchanged.
    assert(pulse.command(brake,true,true,11.17).brake==1);
    pulse.command({},false,true,11.18);
    pulse.command(brake,true,true,11.19);
    assert(pulse.command(brake,true,true,11.4).brake==1); // Control gap aborts pulse.
    pulse.command({},false,true,11.41);
    pulse.command(brake,true,true,11.42);
    assert(pulse.command(brake,true,true,11.0).throttle==0); // Rewind cannot replay.
    pulse.duration_s=.005; assert(!pulse.valid_settings());
    pulse.reset(); pulse.duration_s=.01;
    assert(pulse.valid_settings());
    pulse.command({},false,true,20);
    assert(pulse.command(brake,true,true,20.01).throttle==.08);
    assert(pulse.command(brake,true,true,20.02).brake==1);
    assert(pulse.command(brake,true,true,20.03).brake==1);
    pulse.duration_s=.05; pulse.throttle=0; assert(!pulse.valid_settings());
    pulse.throttle=std::numeric_limits<double>::quiet_NaN(); assert(!pulse.valid_settings());
  }
  std::cout<<"joy manager continuous input / button edges: passed\n";
}
