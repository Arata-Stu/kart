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
  assert(unguarded_actions->deadman && !unguarded_actions->stop); // Release does not revoke permission.
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
  std::cout<<"joy manager continuous input / button edges: passed\n";
}
