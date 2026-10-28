#include "kart_control/lateral_mpc.hpp"
#include <cassert>
#include <iostream>
using namespace kart_control;
int main(){
 Settings c{.257,.384,.8,5.,1.,.1,1.}; MpcSettings m;
 std::vector<Point> straight;for(int i=0;i<301;++i)straight.push_back({i*.1,0,2.});
 auto r=lateral_mpc(straight,false,{1,.2,0},c,m,2.,0.,1./30,100);
 std::cout<<r.reason<<" "<<r.iterations<<" "<<r.command.steering<<"\n";
 assert(r.command.valid && r.command.steering<0 && std::abs(r.command.steering)<=.05+1e-9);
 assert(!lateral_mpc(straight,false,{1,2,0},c,m,2,0,1./30).command.valid);
 assert(!lateral_mpc(straight,false,{1,0,0},c,m,-1,0,1./30).command.valid);
 assert(!lateral_mpc(straight,false,{1,0,0},c,m,NAN,0,1./30).command.valid);
 assert(!lateral_mpc(straight,false,{1,0,0},c,m,1,0,1./30,1e-9).command.valid);
 auto stop=lateral_mpc(straight,false,{29.99,0,0},c,m,0,0,1./30,100);
 assert(stop.command.valid && stop.command.speed==0);
 for(double speed:{.5,1.,2.,3.}) {
   Pose p{1,.3,0}; double previous=0;
   for(int i=0;i<300;++i){
     auto a=lateral_mpc(straight,false,p,c,m,speed,previous,1./30,100);
     if(!a.command.valid)std::cerr<<a.reason<<" speed "<<speed<<"\n";
     assert(a.command.valid);
     assert(std::abs(a.command.steering-previous)<=m.steering_rate/30+1e-9);
     previous=a.command.steering;
     p.x+=speed*std::cos(p.yaw)/30;p.y+=speed*std::sin(p.yaw)/30;p.yaw+=speed*std::tan(previous)/c.wheelbase/30;
   }
   std::cout<<speed<<" final error "<<p.y<<"\n";
   assert(std::abs(p.y)<.03);
 }
 std::vector<Point> circle;for(int i=0;i<400;++i){double a=i*2*3.141592653589793/400;circle.push_back({3*std::sin(a),3-3*std::cos(a),2});}
 Pose p{0,0,0};double previous=0;
 for(int i=0;i<400;++i){auto a=lateral_mpc(circle,true,p,c,m,2,previous,1./30,100);assert(a.command.valid);previous=a.command.steering;p.x+=2*std::cos(p.yaw)/30;p.y+=2*std::sin(p.yaw)/30;p.yaw+=2*std::tan(previous)/c.wheelbase/30;assert(std::abs(std::hypot(p.x,p.y-3)-3)<.15);}
 // Same MPC settings on varying curvature, compared to existing PP on one path.
 std::vector<Point> ellipse;
 for(int i=0;i<600;++i){double a=i*2*3.141592653589793/600;ellipse.push_back({4*std::sin(a),3-3*std::cos(a),2});}
 for(double v:{1.,2.,3.}) {
   Pose mp{0,0,0},pp=mp;double last=0,max_error=0,max_ms=0;
   for(int i=0;i<350;++i) {
     auto a=lateral_mpc(ellipse,true,mp,c,m,v,last,1./30,100);
     auto b=pursue(ellipse,true,pp,c);assert(a.command.valid && b.valid);
     assert(std::abs(a.command.steering)<=c.steering_limit);
     assert(std::abs(a.command.steering-last)<=m.steering_rate/30+1e-9);
     last=a.command.steering;max_ms=std::max(max_ms,a.elapsed_ms);
     mp.x+=v*std::cos(mp.yaw)/30;mp.y+=v*std::sin(mp.yaw)/30;mp.yaw+=v*std::tan(last)/c.wheelbase/30;
     pp.x+=v*std::cos(pp.yaw)/30;pp.y+=v*std::sin(pp.yaw)/30;pp.yaw+=v*std::tan(b.steering)/c.wheelbase/30;
     double error=1e9;for(auto point:ellipse)error=std::min(error,std::hypot(point.x-mp.x,point.y-mp.y));
     max_error=std::max(max_error,error);
   }
   std::cout<<"ellipse "<<v<<" m/s max_error="<<max_error<<" solve_max_ms="<<max_ms<<"\n";
   assert(max_error<.15);
 }
 MpcSettings bad=m;bad.horizon=3;assert(!valid_mpc(bad));
 auto duplicate=straight;duplicate[1]=duplicate[0];
 assert(!lateral_mpc(duplicate,false,{1,0,0},c,m,1,0,1./30).command.valid);
 assert(!lateral_mpc(straight,false,{1,0,1},c,m,1,0,1./30).command.valid);
 auto stopped_line=straight;stopped_line[20].speed=0;
 auto braking=lateral_mpc(stopped_line,false,{1,0,0},c,m,1,0,1./30,100);
 assert(braking.command.valid && braking.command.speed<=std::sqrt(2*c.deceleration));
 // Step references switch at the boundary, not across the preceding segment.
 auto step_line=straight;for(auto& point:step_line)point.speed=point.x<2?2:.5;
 c.step_speed=true;
 for(double x:{1.95,2.,2.05}) {
   auto pp=pursue(step_line,false,{x,0,0},c);
   auto mp=lateral_mpc(step_line,false,{x,0,0},c,m,1,0,1./30,100);
   assert(pp.valid && mp.command.valid);
   assert(pp.speed==(x<2?2:.5));assert(mp.command.speed==(x<2?2:.5));
 }
 c.step_speed=false;
 assert(pursue(step_line,false,{1.95,0,0},c).speed<2);
 std::cout<<"MPC closed-loop tests passed\n";
}
