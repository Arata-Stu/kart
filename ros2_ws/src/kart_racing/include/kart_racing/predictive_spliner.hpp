#pragma once
// Independent kart implementation, inspired by arXiv:2410.04868.
// Local Gaussian processes + bounded spline candidate search (NOT paper SQP).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace kart_racing {
inline bool finite(double v) { return std::isfinite(v); }
inline double wrap(double s,double length) { return s-std::floor(s/length)*length; }
struct Prediction { double d=0,v=0,d_sigma=1,v_sigma=1; bool valid=false; };
struct Bin { double d=0,v=0,d_noise=0,v_noise=0; unsigned n=0; };
class OpponentModel {
  double length_,step_,first_=0,last_=0,stamp_=-1;
  std::uint64_t id_=0;
  std::vector<Bin> bins_;
  // Exact posterior over at most 20 nearby observed bins. Fixed bounded work.
  double kernel(double distance,bool lateral) const {
    const double chord=length_/3.141592653589793*std::sin(3.141592653589793*distance/length_);
    const double z=chord/1.5;
    return lateral ? (1+std::sqrt(3.)*z)*std::exp(-std::sqrt(3.)*z) : std::exp(-.5*z*z);
  }
  std::pair<double,double> regress(double s,bool lateral) const {
    std::vector<std::pair<double,size_t>> near;
    for(size_t i=0;i<bins_.size();++i) if(bins_[i].n) {
      double r=std::abs(wrap(s,length_)-(i+.5)*step_);r=std::min(r,length_-r);
      near.emplace_back(r,i);
    }
    std::sort(near.begin(),near.end());if(near.size()>20)near.resize(20);
    if(near.empty() || near.front().first>2*step_)return {0,1};
    const size_t n=near.size();double mean=0;
    for(auto a:near)mean+=lateral?bins_[a.second].d:bins_[a.second].v;
    mean/=n;
    std::vector<double> l(n*n,0),k(n),y(n),z(n);
    for(size_t i=0;i<n;++i){
      const auto& b=bins_[near[i].second];k[i]=kernel(near[i].first,lateral);
      y[i]=(lateral?b.d:b.v)-mean;
      for(size_t j=0;j<=i;++j){
        double r=std::abs(static_cast<double>(near[i].second)-near[j].second)*step_;
        r=std::min(r,length_-r);
        double a=kernel(r,lateral)+(i==j? .0025+(lateral?b.d_noise:b.v_noise):0);
        for(size_t h=0;h<j;++h)a-=l[i*n+h]*l[j*n+h];
        l[i*n+j]=i==j?std::sqrt(std::max(a,1e-9)):a/l[j*n+j];
      }
      for(size_t j=0;j<i;++j)y[i]-=l[i*n+j]*y[j];
      y[i]/=l[i*n+i];
      z[i]=k[i];for(size_t j=0;j<i;++j)z[i]-=l[i*n+j]*z[j];z[i]/=l[i*n+i];
    }
    double value=mean,variance=1;
    for(size_t i=0;i<n;++i){value+=z[i]*y[i];variance-=z[i]*z[i];}
    return {value,std::sqrt(std::max(.0025,variance))};
  }
public:
  explicit OpponentModel(double length):length_(length) {
    if(!finite(length)||length<3||length>20000)throw std::invalid_argument("course_length");
    bins_.resize(std::min<size_t>(2048,static_cast<size_t>(std::ceil(length/.5))));
    step_=length/bins_.size();
  }
  void clear(){for(auto& b:bins_)b={};stamp_=-1;first_=last_=0;}
  double coverage() const {size_t n=0;for(auto b:bins_)n+=b.n>0;return double(n)/bins_.size();}
  bool ready() const{return coverage()>=.9 && last_-first_>=length_*.95;}
  Prediction predict(double s) const {
    if(!finite(s))return {};
    auto d=regress(s,true),v=regress(s,false);
    return {d.first,std::max(0.,v.first),d.second,v.second,d.second<.35&&v.second<.35};
  }
  bool observe(std::uint64_t id,double time,double s,double d,double v,double dv,double vv){
    if(!finite(time)||!finite(s)||!finite(d)||!finite(v)||!finite(dv)||!finite(vv)||v<0||v>20||dv<0||vv<0||dv>.25||vv>4)return false;
    if(id!=id_){clear();id_=id;}
    if(stamp_>=0 && time<=stamp_) {if(time<stamp_)clear();return false;}
    if(stamp_>=0 && (time-stamp_>.5 || s<last_-.2 || s-last_>20*(time-stamp_)+.5))clear();
    if(ready()) {auto p=predict(s);if(p.valid&&(std::abs(d-p.d)>.2+3*p.d_sigma||std::abs(v-p.v)>.5+3*p.v_sigma))clear();}
    if(stamp_<0)first_=s;
    stamp_=time;last_=s;
    auto& b=bins_[std::min(bins_.size()-1,static_cast<size_t>(wrap(s,length_)/step_))];
    // Exponential bounded history adapts to new laps; uncertainty is not divided
    // by frame count, because temporally adjacent observations are correlated.
    double a=1./std::min(10u,b.n+1);b.d+=a*(d-b.d);b.v+=a*(v-b.v);
    b.d_noise=dv;b.v_noise=vv;b.n=std::min(10u,b.n+1);return true;
  }
};
struct Settings {
  double length=.35,width=.19,margin=.08,min_gap=1,headway=.7;
  double acceleration=1.5,deceleration=2.5,curvature=1.2,horizon=15;
  bool valid() const {
    for(double x:{length,width,margin,min_gap,headway,acceleration,deceleration,curvature,horizon})if(!finite(x)||x<=0)return false;
    return length<=2&&width<=1&&horizon<=30&&curvature<=10;
  }
};
struct Scene {
  double time=0,ego_s=0,ego_d=0,ego_v=0,opponent_s=0,opponent_d=0,opponent_v=0;
  double d_variance=0,v_variance=0,speed_cap=0,zone_end=0,left_width=0,right_width=0;
  std::uint64_t track_id=1;
  bool observed=true,passing_allowed=false;
};
struct Plan {
  bool valid=false;double start=0,end=0,ramp=0,offset=0,roc_start=0,roc_end=0;
  double at(double s) const {
    if(!valid)return 0;
    double u=std::clamp(std::min((s-start)/ramp,(end-s)/ramp),0.,1.);
    return offset*(10*u*u*u-15*u*u*u*u+6*u*u*u*u*u);
  }
};
struct Output { bool valid=false;double speed=0,coverage=0;std::string state="FAULT",reason="invalid_input";Plan plan; };
class Planner {
  Settings cfg_;OpponentModel model_;Plan active_;double last_time_=-1,pass_cap_=0;std::uint64_t id_=0;bool passed_=false;
public:
  Planner(double length,Settings settings={}):cfg_(settings),model_(length){if(!cfg_.valid())throw std::invalid_argument("settings");}
  const OpponentModel& model()const{return model_;}
  void reset(){model_.clear();active_={};last_time_=-1;passed_=false;}
  Output update(const Scene& a){
    Output out;
    for(double x:{a.time,a.ego_s,a.ego_d,a.ego_v,a.opponent_s,a.opponent_d,a.opponent_v,a.d_variance,a.v_variance,a.speed_cap,a.zone_end,a.left_width,a.right_width})if(!finite(x)){reset();return out;}
    if(!a.observed || a.ego_v<0 || a.ego_v>20 || a.speed_cap<0 || a.speed_cap>20 || (last_time_>=0 && a.time<=last_time_)) {reset();out.reason="observation_or_clock_invalid";return out;}
    if(last_time_>=0 && (a.time-last_time_>.5 || a.track_id!=id_)) {reset();out.reason="observation_discontinuity";return out;}
    last_time_=a.time;id_=a.track_id;
    if(!model_.observe(a.track_id,a.time,a.opponent_s,a.opponent_d,a.opponent_v,a.d_variance,a.v_variance)){reset();return out;}
    out.coverage=model_.coverage();out.valid=true;out.state="FOLLOW_OBSERVE";out.reason="learning";
    const double gap=a.opponent_s-a.ego_s-cfg_.length;
    if(passed_){
      out.valid=a.ego_s-a.opponent_s-cfg_.length>=cfg_.min_gap;
      out.speed=out.valid?a.speed_cap:0;out.state=out.valid?"PASSED":"FAULT";
      out.reason=out.valid?"complete":"opponent_reapproached";return out;
    }
    out.speed=std::min({a.speed_cap,std::max(0.,a.opponent_v+.8*(gap-cfg_.min_gap-cfg_.headway*a.ego_v)),std::sqrt(std::max(0.,a.opponent_v*a.opponent_v+2*cfg_.deceleration*(gap-cfg_.min_gap)))});
    if(active_.valid){
      if(!model_.ready() || !a.passing_allowed || a.speed_cap<pass_cap_-.05 || active_.end>a.zone_end || active_.offset+cfg_.width/2+cfg_.margin>a.left_width || active_.offset-cfg_.width/2-cfg_.margin< -a.right_width){active_={};out.valid=false;out.speed=0;out.state="FAULT";out.reason="pass_prediction_invalid";return out;}
      if(a.ego_s>=active_.end){
        if(a.ego_s-a.opponent_s-cfg_.length>=cfg_.min_gap && std::abs(a.ego_d)<.15){passed_=true;out.state="PASSED";out.reason="complete";out.speed=a.speed_cap;active_={};return out;}
        out.valid=false;out.speed=0;out.state="FAULT";out.reason="merge_incomplete";active_={};return out;
      }
      // Never command an inward merge while still alongside the opponent.
      if(a.ego_s>=active_.end-active_.ramp && a.ego_s-a.opponent_s-cfg_.length<cfg_.min_gap){out.valid=false;out.speed=0;out.state="FAULT";out.reason="merge_unavailable";active_={};return out;}
      out.plan=active_;out.speed=a.speed_cap;out.state=a.ego_s>=active_.end-active_.ramp?"MERGE":"PASS";out.reason="prediction_valid";return out;
    }
    if(!model_.ready())return out;
    if(!a.passing_allowed){out.reason="passing_not_allowed";return out;}
    out.state="OPPORTUNITY_READY";out.reason="no_feasible_trajectory";
    if(a.speed_cap-a.opponent_v<.5){out.reason="speed_advantage_insufficient";return out;}
    if(gap<cfg_.min_gap){out.reason="following_gap_insufficient";return out;}
    if(std::abs(a.ego_d)>.15){out.reason="ego_off_reference";return out;}
    // Propagate learned opponent speed with an upper uncertainty bound.
    double es=a.ego_s,os=a.opponent_s,ev=a.ego_v,begin=-1,end=-1,max_d=-1e9,min_d=1e9;
    for(double t=0;t<cfg_.horizon;t+=.1){
      auto p=model_.predict(os);if(!p.valid){out.reason="prediction_unavailable";return out;}
      const double pv=p.v+2*p.v_sigma;
      if(a.speed_cap-pv<.2){out.reason="predicted_speed_advantage_insufficient";return out;}
      ev=std::min(a.speed_cap,ev+cfg_.acceleration*.1);es+=ev*.1;os+=pv*.1;
      if(es+cfg_.length+cfg_.margin>=os){
        if(begin<0)begin=es;
        max_d=std::max(max_d,p.d+2*p.d_sigma);min_d=std::min(min_d,p.d-2*p.d_sigma);
        if(es-os>=cfg_.length+cfg_.min_gap+cfg_.headway*pv){end=es;break;}
      }
    }
    if(begin<0||end<0){out.reason="prediction_horizon_insufficient";return out;}
    bool longitudinal_candidate=false;
    double best=std::numeric_limits<double>::infinity();
    // Bounded candidate optimization, conservatively restricted to straight
    // passing corridors. GP inference is outside this search.
    for(int side:{-1,1})for(double stretch:{1.,1.25,1.5}){
      double offset=side>0?max_d+cfg_.width+cfg_.margin:min_d-cfg_.width-cfg_.margin;
      double ramp=std::max({3.,a.speed_cap,std::sqrt(6*std::abs(offset)/cfg_.curvature)})*stretch;
      Plan p{true,a.ego_s,end+ramp,ramp,offset,begin,end};
      if(begin-a.ego_s<ramp || p.end>a.zone_end || p.end-p.start<2*ramp)continue;
      longitudinal_candidate=true;
      if(offset+cfg_.width/2+cfg_.margin>a.left_width || offset-cfg_.width/2-cfg_.margin< -a.right_width)continue;
      const double score=std::abs(offset)*(p.end-p.start)+2/ramp;
      if(score<best){best=score;active_=p;pass_cap_=a.speed_cap;}
    }
    if(active_.valid){out.plan=active_;out.speed=a.speed_cap;out.state="PASS";out.reason="learned_prediction";}
    else out.reason=longitudinal_candidate?"lateral_clearance_insufficient":"longitudinal_space_insufficient";
    return out;
  }
};
} // namespace kart_racing
