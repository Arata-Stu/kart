#include "kart_control/lateral_mpc.hpp"
#include "kart_control/lookahead.hpp"
#include <iostream>
#include <iomanip>
using namespace kart_control;
int main(){
 size_t n;int closed,mpc,step;
 Settings c{.257,.384,.8,1.5,2.,.04,2.5};MpcSettings m;
 if(!(std::cin>>n>>closed>>mpc>>step>>c.max_speed>>c.wheelbase>>c.steering_limit>>m.horizon>>m.tracking>>m.smoothness>>m.steering_rate)||n<2||n>5000)return 2;
 const bool stopped=c.max_speed==0;
 if(stopped)c.max_speed=.001;
 c.step_speed=step;
 std::vector<Point> line(n);for(auto& p:line)if(!(std::cin>>p.x>>p.y>>p.speed))return 2;
 Pose p{line[0].x,line[0].y,std::atan2(line[1].y-line[0].y,line[1].x-line[0].x)};
 double v=0,steer=0,travel=0,total=0,maxerror=0,still=0;const double dt=1./30;
 for(size_t i=0;i<n-(closed?0:1);++i)total+=std::hypot(line[(i+1)%n].x-line[i].x,line[(i+1)%n].y-line[i].y);
 std::cout<<std::setprecision(12);
 for(int tick=0;tick<9000;++tick){
   Result cmd;double ms=0;
   if(mpc){auto result=lateral_mpc(line,closed,p,c,m,v,steer,dt);cmd=result.command;ms=result.elapsed_ms;if(!cmd.valid){std::cout<<"END "<<result.reason<<"\n";return 0;}}
   else {
     c.lookahead=*lookahead_distance(.8,v,LookaheadSettings{});
     cmd=pursue(line,closed,p,c);
   }
   if(!cmd.valid){std::cout<<"END invalid_geometry_or_off_path\n";return 0;}
   if(stopped)cmd.speed=0;
   double error=1e20;
   for(size_t i=0;i<n-(closed?0:1);++i){auto a=line[i],b=line[(i+1)%n];double dx=b.x-a.x,dy=b.y-a.y;
     double f=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/std::max(1e-12,dx*dx+dy*dy),0.,1.);
     error=std::min(error,std::hypot(p.x-a.x-f*dx,p.y-a.y-f*dy));}
   maxerror=std::max(error,maxerror);
   std::cout<<tick*dt<<' '<<p.x<<' '<<p.y<<' '<<p.yaw<<' '<<v<<' '<<cmd.steering<<' '<<error<<' '<<maxerror<<' '<<cmd.target_x<<' '<<cmd.target_y<<' '<<ms<<' '<<cmd.speed<<' '<<(mpc?0:c.lookahead)<<'\n';
   if((closed&&travel>=total)||(!closed&&std::hypot(p.x-line.back().x,p.y-line.back().y)<c.goal_tolerance)){std::cout<<"END completed\n";return 0;}
   still=v<.01?still+dt:0;if(still>=5){std::cout<<"END stopped_target\n";return 0;}
   steer=cmd.steering;v+=std::clamp(cmd.speed-v,-2.5*dt,1.5*dt);
   double turn=v/c.wheelbase*std::tan(steer)*dt;
   p.x+=v*std::cos(p.yaw+turn/2)*dt;p.y+=v*std::sin(p.yaw+turn/2)*dt;p.yaw+=turn;travel+=v*dt;
 }
 std::cout<<"END time_limit\n";
}
