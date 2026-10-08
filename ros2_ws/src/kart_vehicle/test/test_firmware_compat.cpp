// Optional interoperability test against the user's actual firmware checkout.
#include "kart_vehicle/bridge_protocol.hpp"
extern "C" {
#include "jpbb_protocol.h"
#include "jpbb_board.h"
void jpbb_status_snapshot(jpbb_status_t *status) {*status={};}
void jpbb_pwm_get(uint16_t *servo,uint16_t *esc) {*servo=1500; *esc=1500;}
}
#include <cassert>
#include <iostream>
using namespace kart_vehicle;
int main() {
  for (int steering : {-1000,-500,0,500,1000}) {
    for (int speed : {0,300,1000}) {
      for (int direction=0;direction<3;++direction) {
        CommandFrame frame; frame.sequence=0xffffffffU; frame.steering_milli=steering; frame.flags=3;
        if(direction==0) {frame.throttle_milli=speed;}
        if(direction==1) {frame.reverse_milli=speed;}
        if(direction==2) {frame.brake_milli=speed;}
        const auto wire=encode_command(frame); jpbb_command_t parsed{};
        assert(jpbb_protocol_parse(wire.c_str(),&parsed));
        assert(parsed.kind==JPBB_COMMAND_HOST_FRAME && parsed.flags==3 && parsed.sequence==0xffffffffU);
        assert(parsed.steering_milli==steering && parsed.throttle_milli==frame.throttle_milli &&
               parsed.reverse_milli==frame.reverse_milli && parsed.brake_milli==frame.brake_milli);
        uint16_t servo=0,esc=0; assert(jpbb_protocol_host_pwm(&parsed,&servo,&esc));
        assert(servo==1500+steering/2); assert(esc==(direction==0 ? 1500-speed/2 : 1500+speed/2));
      }
    }
  }
  std::cout<<"45 host command frames parsed by actual firmware, PWM polarity: passed\n";
}
