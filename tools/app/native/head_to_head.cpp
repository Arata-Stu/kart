// Stateless local-route adapter to the production lateral controllers.
#include "kart_control/lateral_mpc.hpp"
#include "kart_control/lookahead.hpp"
#include <iostream>
#include <iomanip>
using namespace kart_control;
int main() {
  int method,step; Settings c{.257,.384,.8,1.5,2.,.04,2.5}; MpcSettings m;
  if (!(std::cin>>method>>step>>c.wheelbase>>c.steering_limit>>m.horizon>>m.tracking>>m.smoothness>>m.steering_rate)) return 2;
  c.step_speed=step;
  std::cout<<std::setprecision(12);
  size_t n; Pose p; double v,steer;
  while(std::cin>>n>>p.x>>p.y>>p.yaw>>v>>steer>>c.max_speed) {
    if(n<2 || n>1000) return 2;
    std::vector<Point> line(n);
    for(auto& a:line) if(!(std::cin>>a.x>>a.y>>a.speed))return 2;
    c.max_speed=std::max(.001,c.max_speed);
    Result cmd; double ms=0; std::string reason="ok";
    c.lookahead=*lookahead_distance(.8,v,LookaheadSettings{});
    if(method) {auto r=lateral_mpc(line,false,p,c,m,v,steer,1./30);cmd=r.command;ms=r.elapsed_ms;reason=r.reason;}
    else {cmd=pursue(line,false,p,c);if(!cmd.valid)reason="invalid_geometry_or_off_path";}
    std::cout<<cmd.valid<<' '<<cmd.steering<<' '<<cmd.speed<<' '<<cmd.target_x<<' '<<cmd.target_y<<' '<<ms<<' '<<(method?0:c.lookahead)<<' '<<(reason.empty()?"invalid":reason)<<std::endl;
  }
}
