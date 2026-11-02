#include "kart_racing/predictive_spliner.hpp"
#include <iostream>
#include <iomanip>
int main(){
  double length;kart_racing::Settings c;
  if(!(std::cin>>length>>c.length>>c.width>>c.margin>>c.min_gap>>c.headway>>c.curvature))return 2;
  try{
    kart_racing::Planner planner(length,c);kart_racing::Scene a;
    std::cout<<std::setprecision(12);
    while(std::cin>>a.time>>a.ego_s>>a.ego_d>>a.ego_v>>a.opponent_s>>a.opponent_d>>a.opponent_v>>a.speed_cap>>a.zone_end>>a.left_width>>a.right_width>>a.passing_allowed){
      auto r=planner.update(a);
      std::cout<<r.valid<<' '<<r.speed<<' '<<r.coverage<<' '<<r.state<<' '<<r.reason<<' '<<r.plan.valid<<' '<<r.plan.start<<' '<<r.plan.end<<' '<<r.plan.ramp<<' '<<r.plan.offset<<' '<<r.plan.roc_start<<' '<<r.plan.roc_end<<std::endl;
    }
  }catch(const std::exception&){return 2;}
}
