#pragma once
#include "kart_control/pure_pursuit.hpp"
#include <array>
#include <chrono>

namespace kart_control {
struct MpcSettings {
  double horizon{1.0}, tracking{10.0}, smoothness{1.0}, steering_rate{1.5};
};
inline bool valid_mpc(MpcSettings m) {
  return std::isfinite(m.horizon) && m.horizon>=.3 && m.horizon<=2.0 &&
    std::isfinite(m.tracking) && m.tracking>=.1 && m.tracking<=100 &&
    std::isfinite(m.smoothness) && m.smoothness>=.1 && m.smoothness<=100 &&
    std::isfinite(m.steering_rate) && m.steering_rate>=.1 && m.steering_rate<=5;
}
inline double angle_wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
struct MpcResult { Result command; int iterations{}; double elapsed_ms{}; const char *reason{"mpc_invalid_input"}; };

// Fixed-size condensed convex QP. Linearized forward bicycle model in path-error
// coordinates. ADMM handles BOTH angle and first-difference box constraints.
// No external solver, no warm-start or stale solution fallback.
inline MpcResult lateral_mpc(const std::vector<Point>& line, bool closed, Pose pose,
    Settings c, MpcSettings m, double speed, double previous, double command_dt,
    double budget_ms=10.0) {
  const auto start=std::chrono::steady_clock::now();
  MpcResult out;
  auto finish=[&](const char* why) {
    out.reason=why;
    out.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    if(out.elapsed_ms>budget_ms) {out.command={};out.reason="mpc_timeout";}
    return out;
  };
  if(!valid_mpc(m) || !std::isfinite(speed) || speed<-.05 || !std::isfinite(previous) ||
     !std::isfinite(command_dt) || command_dt<=0 || command_dt>.2 ||
     !std::isfinite(budget_ms) || budget_ms<=0 || line.size()<(closed?3U:2U)) return finish("mpc_invalid_input");
  for(double v:{c.wheelbase,c.steering_limit,c.max_speed,c.max_error,c.goal_tolerance,c.deceleration})
    if(!std::isfinite(v)||v<=0) return finish("mpc_invalid_input");
  if(c.steering_limit>=1.3 || std::abs(previous)>c.steering_limit+1e-8) return finish("mpc_invalid_input");
  for(double v:{pose.x,pose.y,pose.yaw}) if(!std::isfinite(v)) return finish("mpc_invalid_input");
  const size_t segments=closed?line.size():line.size()-1;
  std::vector<double> arc(segments+1,0);
  double best=1e100, origin=0;
  for(const auto& p:line) if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.speed)||p.speed<0) return finish("mpc_invalid_input");
  for(size_t i=0;i<segments;++i) {
    auto a=line[i],b=line[(i+1)%line.size()]; double dx=b.x-a.x,dy=b.y-a.y,l=std::hypot(dx,dy);
    if(l<1e-6) return finish("mpc_invalid_input");
    arc[i+1]=arc[i]+l;
    double t=std::clamp(((pose.x-a.x)*dx+(pose.y-a.y)*dy)/(l*l),0.,1.);
    double d=std::hypot(pose.x-a.x-t*dx,pose.y-a.y-t*dy);
    if(d<best){best=d;origin=arc[i]+t*l;}
  }
  if(best>c.max_error) return finish("mpc_off_path");
  auto sample=[&](double s) {
    s=closed?std::fmod(std::fmod(s,arc.back())+arc.back(),arc.back()):std::clamp(s,0.,arc.back());
    size_t i=std::min(segments-1,size_t(std::upper_bound(arc.begin(),arc.end(),s)-arc.begin()-1));
    double t=(s-arc[i])/(arc[i+1]-arc[i]);auto a=line[i],b=line[(i+1)%line.size()];
    return Point{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),(c.step_speed ? (t>=1.-1e-8?b.speed:a.speed) : a.speed+t*(b.speed-a.speed))};
  };
  auto heading=[&](double s) {if(!closed)s=std::clamp(s,0.,arc.back());auto a=sample(s-.05),b=sample(s+.05);return std::atan2(b.y-a.y,b.x-a.x);};
  auto ref=sample(origin);double yaw=heading(origin);
  if(std::abs(angle_wrap(pose.yaw-yaw))>.7) return finish("mpc_heading_error");
  if(!closed && arc.back()-origin<=c.goal_tolerance) {
    out.command={previous,0,line.back().x,line.back().y,true};return finish("zero_speed_target");
  }
  // Keep longitudinal PID unchanged. Reference speed uses the nearest point and
  // forward braking envelope; unlike PP it does not depend on a lookahead knob.
  double target=std::min(c.max_speed,ref.speed);
  for(size_t i=0;!c.step_speed && i<line.size();++i) {
    double d=arc[i]-origin;
    if(closed && d<0) d+=arc.back();
    if(d>=0) target=std::min(target,std::sqrt(line[i].speed*line[i].speed+2*c.deceleration*d));
  }
  if(!closed) target=std::min(target,std::sqrt(2*c.deceleration*std::max(0.,arc.back()-origin-c.goal_tolerance)));
  // Startup uses a small planning speed; measured forward speed otherwise.
  const double v=std::max(.15,speed), dt=m.horizon/20., ds=v*dt;
  constexpr int n=20;
  using Vec=std::array<double,n>;using Mat=std::array<Vec,n>;
  Mat H{},L{};Vec g{},u{},z{},w{},y{},q{},ey{},ep{};
  double e=-std::sin(yaw)*(pose.x-ref.x)+std::cos(yaw)*(pose.y-ref.y);
  double p=angle_wrap(pose.yaw-yaw);
  for(int k=0;k<n;++k) {
    double next_yaw=heading(origin+(k+1)*ds), turn=angle_wrap(next_yaw-yaw);
    double ff=std::atan(c.wheelbase*turn/ds);
    // tan(delta) linearized around curvature feedforward, not around zero.
    double B=ds/c.wheelbase/(std::cos(ff)*std::cos(ff));
    e+=ds*p;for(int j=0;j<n;++j) ey[j]+=ds*ep[j];
    p+=ds/c.wheelbase*std::tan(ff)-B*ff-turn;ep[k]+=B;
    for(int i=0;i<n;++i) {
      g[i]+=m.tracking*(ey[i]*e+.4*ep[i]*p);
      for(int j=0;j<n;++j) H[i][j]+=m.tracking*(ey[i]*ey[j]+.4*ep[i]*ep[j]);
    }
    H[k][k]+=.05;g[k]-=.05*ff;
    H[k][k]+=m.smoothness;
    if(k){H[k-1][k-1]+=m.smoothness;H[k][k-1]-=m.smoothness;H[k-1][k]-=m.smoothness;}
    else g[0]-=m.smoothness*previous;
    yaw=next_yaw;
  }
  const double rho=10.;
  L=H;
  for(int i=0;i<n;++i) {
    L[i][i]+=rho*(i+1<n?3:2);
    if(i){L[i][i-1]-=rho;L[i-1][i]-=rho;}
  }
  // Cholesky of H + rho*(I + D'D).
  for(int i=0;i<n;++i) for(int j=0;j<=i;++j) {
    double a=L[i][j];for(int k=0;k<j;++k)a-=L[i][k]*L[j][k];
    if(i==j){if(!(a>0)||!std::isfinite(a))return finish("mpc_factorization_failed");L[i][j]=std::sqrt(a);}
    else L[i][j]=a/L[j][j];
  }
  u.fill(previous);z=u;w[0]=previous;
  bool converged=false;
  for(int iteration=0;iteration<1500;++iteration) {
    Vec rhs{},tmp{},oldz=z,oldw=w;
    for(int i=0;i<n;++i) rhs[i]=-g[i]+rho*(z[i]-y[i]+w[i]-q[i]-(i+1<n?w[i+1]-q[i+1]:0));
    for(int i=0;i<n;++i){double a=rhs[i];for(int j=0;j<i;++j)a-=L[i][j]*tmp[j];tmp[i]=a/L[i][i];}
    for(int i=n-1;i>=0;--i){double a=tmp[i];for(int j=i+1;j<n;++j)a-=L[j][i]*u[j];u[i]=a/L[i][i];}
    double residual=0,dual=0;
    for(int i=0;i<n;++i) {
      double d=u[i]-(i?u[i-1]:0), center=i?0:previous;
      double limit=m.steering_rate*(i?dt:command_dt);
      z[i]=std::clamp(u[i]+y[i],-c.steering_limit,c.steering_limit);
      w[i]=std::clamp(d+q[i],center-limit,center+limit);
      y[i]+=u[i]-z[i];q[i]+=d-w[i];
      residual=std::max({residual,std::abs(u[i]-z[i]),std::abs(d-w[i])});
      dual=std::max({dual,rho*std::abs(z[i]-oldz[i]),rho*std::abs(w[i]-oldw[i])});
    }
    out.iterations=iteration+1;
    if(residual<1e-5 && dual<1e-4){converged=true;break;}
    if(iteration%16==0 && std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()>budget_ms) return finish("mpc_timeout");
  }
  if(!converged) return finish("mpc_not_converged");
  auto end=sample(origin+v*m.horizon);
  // Remove numerical tolerance from the first command's hard constraints.
  double steering=std::clamp(u[0],std::max(-c.steering_limit,previous-m.steering_rate*command_dt),std::min(c.steering_limit,previous+m.steering_rate*command_dt));
  if(!std::isfinite(steering)) return finish("mpc_nonfinite");
  out.command={steering,target,end.x,end.y,true};return finish(target>0?"tracking":"zero_speed_target");
}
} // namespace kart_control
