#include "kart_racing/predictive_spliner.hpp"
#include <cassert>
#include <iostream>
using namespace kart_racing;
int main(){
  OpponentModel gp(40);
  for(int i=0;i<=420;++i)assert(gp.observe(1,i*.05,i*.1,.2,2,.001,.01));
  assert(gp.ready());auto p=gp.predict(39.9);assert(p.valid);assert(std::abs(p.d-.2)<.03);assert(std::abs(p.v-2)<.03);
  assert(gp.predict(.1).valid); // periodic seam
  gp.observe(2,22,44,.2,2,0,0);assert(!gp.ready()); // identity switch
  assert(!gp.observe(2,21,44,.2,2,0,0)); // reversal
  assert(!gp.ready());
  OpponentModel sparse(40);
  for(int i=0;i<200;++i)sparse.observe(1,i*.1,0,0,0,0,0);
  assert(!sparse.ready());assert(!sparse.predict(20).valid);
  Settings cfg;Planner planner(40,cfg);Scene a;a.speed_cap=5;a.opponent_v=2;a.ego_v=2;a.left_width=a.right_width=2;
  Output r;
  for(int i=0;i<=420;++i){a.time=i*.05;a.opponent_s=i*.1;a.ego_s=a.opponent_s-4;r=planner.update(a);assert(r.valid);assert(!r.plan.valid);}
  a.time+=.05;a.opponent_s+=.1;a.ego_s=a.opponent_s-6;a.zone_end=a.ego_s+100;a.passing_allowed=true;
  auto narrow=planner;auto blocked=a;blocked.left_width=blocked.right_width=.15;
  auto nr=narrow.update(blocked);assert(!nr.plan.valid);assert(nr.reason=="lateral_clearance_insufficient");
  auto slow=planner;auto insufficient=a;insufficient.speed_cap=2;
  auto sr=slow.update(insufficient);assert(!sr.plan.valid);assert(sr.reason=="speed_advantage_insufficient");
  auto outside=planner;auto disabled=a;disabled.passing_allowed=false;
  assert(outside.update(disabled).reason=="passing_not_allowed");
  auto short_zone=planner;auto short_scene=a;short_scene.zone_end=a.ego_s+1;
  assert(short_zone.update(short_scene).reason=="longitudinal_space_insufficient");
  r=planner.update(a);assert(r.plan.valid);assert(r.plan.start<r.plan.roc_start);assert(r.plan.end>r.plan.roc_end);
  assert(std::abs(r.plan.at(r.plan.start))<1e-9);assert(std::abs(r.plan.at(r.plan.end))<1e-9);
  assert(std::abs(r.plan.at(r.plan.roc_start))>=cfg.width+cfg.margin);
  Settings tuned_cfg;tuned_cfg.transition_min_distance=1;tuned_cfg.transition_time=.2;
  Planner tuned(40,tuned_cfg);Scene ta;ta.speed_cap=5;ta.opponent_v=2;ta.ego_v=2;ta.left_width=ta.right_width=2;
  for(int i=0;i<=420;++i){ta.time=i*.05;ta.opponent_s=i*.1;ta.ego_s=ta.opponent_s-4;tuned.update(ta);}
  ta.time+=.05;ta.opponent_s+=.1;ta.ego_s=ta.opponent_s-6;ta.zone_end=ta.ego_s+100;ta.passing_allowed=true;
  auto tr=tuned.update(ta);assert(tr.plan.valid);assert(tr.plan.ramp<r.plan.ramp);
  assert(tr.plan.ramp>=std::sqrt(6*std::abs(tr.plan.offset)/tuned_cfg.curvature));
  a.time+=.05;a.observed=false;r=planner.update(a);assert(!r.valid&&r.speed==0&&!r.plan.valid);
  a.observed=true;a.time+=.05;r=planner.update(a);assert(!r.plan.valid&&r.coverage<.1);
  a.time+=.05;a.ego_v=std::numeric_limits<double>::quiet_NaN();assert(!planner.update(a).valid);
  // Stopped rival produces a zero target near the standstill gap.
  Planner stop(40);Scene b;b.speed_cap=5;b.ego_v=1;b.opponent_s=1.35;
  auto stopped=stop.update(b);assert(stopped.valid&&stopped.speed<1e-6);
  std::cout<<"GP, seam, identity, coverage, RoC, spline, loss, NaN, following: passed\n";
}
