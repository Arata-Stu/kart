#include "kart_control/lateral_mpc.hpp"
#include "kart_control/lookahead.hpp"
#include <iostream>
#include <iomanip>
int main(){
 using namespace kart_control;
 size_t n;int mpc;Settings c{.257,.3839724354,.8,1.5,.15,.04,.4};MpcSettings m;
 if(!(std::cin>>n>>mpc>>c.max_speed>>c.wheelbase>>c.steering_limit>>m.horizon>>m.tracking>>m.smoothness>>m.steering_rate)||n<2||n>100000)return 2;
 std::vector<Point> line(n);for(auto& p:line)if(!(std::cin>>p.x>>p.y>>p.speed))return 2;
 size_t begin,end;int parking;double x,y,yaw,speed,previous;
 std::cout<<std::setprecision(12);
 while(std::cin>>begin>>end>>x>>y>>yaw>>speed>>previous>>parking){
  if(begin>=end||end>n)return 2;
  std::vector<Point> window(line.begin()+begin,line.begin()+end);
  c.lookahead=*lookahead_distance(.8,speed,LookaheadSettings{});
  if(parking)c.lookahead=std::min(c.lookahead,.25);
  Result r;double ms=0;
  if(mpc){auto v=lateral_mpc(window,false,{x,y,yaw},c,m,speed,previous,1./30);r=v.command;ms=v.elapsed_ms;}
  else r=pursue(window,false,{x,y,yaw},c);
  std::cout<<r.valid<<' '<<r.steering<<' '<<r.speed<<' '<<r.target_x<<' '<<r.target_y<<' '<<ms<<' '<<c.lookahead<<std::endl;
 }
}
