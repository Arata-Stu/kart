#include "kart_racing/predictive_spliner.hpp"
#include <kart_interfaces/msg/head_to_head_observation.hpp>
#include <kart_interfaces/msg/head_to_head_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <memory>

namespace kart_racing {
class PredictiveSplinerNode:public rclcpp::Node {
  using Observation=kart_interfaces::msg::HeadToHeadObservation;
  using Message=kart_interfaces::msg::HeadToHeadPlan;
  Settings cfg_;double timeout_;std::unique_ptr<Planner> planner_;
  double course_=0;std::string reference_,frame_;Observation::ConstSharedPtr last_;
  std::chrono::steady_clock::time_point received_;
  rclcpp::Subscription<Observation>::SharedPtr sub_;
  rclcpp::Publisher<Message>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double previous_clock_=0;
  Message base(const Observation& a)const{
    Message m;m.header=a.header;m.reference_id=a.reference_id;m.track_id=a.track_id;
    m.valid_for_sec=timeout_;return m;
  }
  void invalidate(const std::string& reason){
    if(planner_)planner_->reset();
    Message m;if(last_)m=base(*last_);m.valid=false;m.state="FAULT";m.reason=reason;pub_->publish(m);
  }
  void receive(Observation::ConstSharedPtr a){
    last_=a;received_=std::chrono::steady_clock::now();
    if(a->header.stamp.sec<0){invalidate("negative_stamp");return;}
    const double now=this->now().seconds(),stamp=rclcpp::Time(a->header.stamp).seconds();
    if(!finite(stamp)||stamp<=0||stamp>now||now-stamp>timeout_||a->reference_id.empty()||a->header.frame_id.empty()){
      invalidate("stale_or_invalid_reference");return;
    }
    if(!finite(a->course_length)||a->course_length<3||a->course_length>20000){invalidate("invalid_course");return;}
    if(!planner_||reference_!=a->reference_id||frame_!=a->header.frame_id||course_!=a->course_length){
      planner_=std::make_unique<Planner>(a->course_length,cfg_);
      reference_=a->reference_id;frame_=a->header.frame_id;course_=a->course_length;
    }
    if(a->measurement_stamp.sec<0){invalidate("negative_measurement_stamp");return;}
    const auto measured=rclcpp::Time(a->measurement_stamp);
    const double epoch=measured.nanoseconds()==0?stamp:measured.seconds();
    if(epoch<stamp||epoch>now){invalidate("invalid_measurement_epoch");return;}
    Scene s;s.time=epoch;s.ego_s=a->ego_s;s.ego_d=a->ego_d;s.ego_v=a->ego_speed;
    s.opponent_s=a->opponent_s;s.opponent_d=a->opponent_d;s.opponent_v=a->opponent_speed;
    s.d_variance=a->opponent_d_variance;s.v_variance=a->opponent_speed_variance;
    s.speed_cap=a->speed_cap;s.zone_end=a->zone_end_s;s.left_width=a->left_width;s.right_width=a->right_width;
    s.track_id=a->track_id;s.observed=a->observed;s.passing_allowed=a->passing_allowed;
    const auto r=planner_->update(s);auto m=base(*a);
    m.valid=r.valid;m.state=r.state;m.reason=r.reason;m.target_speed=r.speed;m.observation_coverage=r.coverage;
    if(r.plan.valid){
      m.roc_start_s=r.plan.roc_start;m.roc_end_s=r.plan.roc_end;
      for(int i=0;i<=100;++i){const double station=r.plan.start+(r.plan.end-r.plan.start)*i/100.;m.stations.push_back(station);m.offsets.push_back(r.plan.at(station));}
    }
    pub_->publish(m);
  }
public:
  explicit PredictiveSplinerNode(const rclcpp::NodeOptions& options):Node("predictive_spliner",options){
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    auto param=[&](const char* name,double value){return declare_parameter<double>(name,value,descriptor);};
    cfg_.length=param("vehicle_length",.35);cfg_.width=param("vehicle_width",.19);
    cfg_.margin=param("margin",.08);cfg_.min_gap=param("min_gap",1);cfg_.headway=param("headway",.7);
    cfg_.acceleration=param("acceleration",1.5);cfg_.deceleration=param("deceleration",2.5);
    cfg_.transition_min_distance=param("transition_min_distance",3.0);
    cfg_.transition_time=param("transition_time",1.0);
    cfg_.curvature=param("max_curvature",1.2);cfg_.horizon=param("prediction_horizon",15);
    cfg_.stationary_speed=param("stationary_speed",.12);
    cfg_.stationary_hold=param("stationary_hold",.7);
    cfg_.avoidance_speed=param("avoidance_speed",1);
    timeout_=param("input_timeout",.2);
    if(!cfg_.valid()||!finite(timeout_)||timeout_<=0||timeout_>.5)throw std::invalid_argument("invalid planner parameters");
    pub_=create_publisher<Message>("racing/shadow_plan",rclcpp::QoS(1).reliable());
    sub_=create_subscription<Observation>("racing/observation",rclcpp::SensorDataQoS().keep_last(1),[this](Observation::ConstSharedPtr a){receive(a);});
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{
      const double now=this->now().seconds();
      if(now<previous_clock_){invalidate("clock_reversed");last_.reset();}
      previous_clock_=now;
      if(!last_)return;
      const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count();
      if(wall>timeout_||now-rclcpp::Time(last_->header.stamp).seconds()>timeout_){invalidate("observation_timeout");last_.reset();}
    });
  }
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(kart_racing::PredictiveSplinerNode)
