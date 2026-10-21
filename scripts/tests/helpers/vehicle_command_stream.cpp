// Portable probe: replay semantic requests through the real HostSession/ESC model.
#include "kart_vehicle/host_session.hpp"
#include <iostream>
int main() {
  kart_vehicle::HostSession session;
  double now; kart_system::Command command;bool first=true;
  while(std::cin>>now>>command.steering>>command.throttle>>command.brake>>command.reverse) {
    if(first) {
      session.start(now-.02);session.output({},true,now-.01,1,0);
      kart_vehicle::StatusFrame status;status.selector=kart_vehicle::RcSelector::automatic;
      status.active_path=kart_vehicle::ActivePath::automatic;session.status(status);first=false;
    }
    const auto frame=session.output(command,true,now,1,0);
    std::cout<<frame.throttle_milli<<' '<<frame.reverse_milli<<' '<<frame.brake_milli<<' '
      <<session.esc().name()<<' '<<static_cast<int>(frame.flags)<<'\n';
  }
}
