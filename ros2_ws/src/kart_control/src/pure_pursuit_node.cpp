#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "kart_control/pure_pursuit.hpp"
#include "kart_interfaces/msg/reference_line.hpp"
#include "kart_interfaces/msg/operation_mode_state.hpp"
#include "kart_interfaces/msg/tracking_reference.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cstdio>
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "kart_control/timing.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace kart_control {
class PurePursuitNode : public rclcpp::Node {
public:
  explicit PurePursuitNode(const rclcpp::NodeOptions &options)
      : Node("pure_pursuit", options) {
    frame_ = param<std::string>("tracking_frame", "rear_axle");
    map_ = param<std::string>("map_frame", "map");
    c_ = {param("wheelbase_m", 0.0),       param("max_steering_rad", 0.0),
          param("lookahead_m", 0.8),       param("max_speed_mps", 10.0),
          param("max_cross_track_m", 1.0), param("goal_tolerance_m", 0.1),
          param("deceleration_mps2", 1.0)};
    wall_timeout_ = param("wall_timeout_s", 0.0);
    if (!std::isfinite(wall_timeout_) || wall_timeout_ < 0)
      throw std::invalid_argument("wall_timeout_s must be finite and nonnegative");
    pose_timeout_ = param("pose_timeout_s", 0.2);
    tf_wait_ = param("tf_lookup_timeout_s", 0.05);
    line_timeout_ = param("line_timeout_s", 2.5);
    localization_timeout_ = param("localization_timeout_s", 2.5);
    double rate = param("control_rate_hz", 30.0);
    if (frame_.empty() || map_.empty() || frame_[0] == '/' || map_[0] == '/' ||
        frame_ == map_)
      throw std::invalid_argument("Invalid TF frames");
    for (double v : {pose_timeout_, line_timeout_, localization_timeout_, rate,
                     c_.lookahead, c_.max_speed, c_.max_error,
                     c_.goal_tolerance, c_.deceleration})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument("Parameters must be finite and positive");
    if (!std::isfinite(tf_wait_) || tf_wait_ < 0 || tf_wait_ > 0.05 ||
        tf_wait_ >= pose_timeout_)
      throw std::invalid_argument("tf_lookup_timeout_s must be in [0,0.05] and below pose_timeout_s");
    if (!std::isfinite(c_.wheelbase) || !std::isfinite(c_.steering_limit) ||
        c_.wheelbase < 0 || c_.steering_limit < 0 || c_.steering_limit >= 1.57)
      throw std::invalid_argument("Invalid vehicle geometry");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // TransformListener's dedicated thread keeps receiving TF while lookup waits.
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    command_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "control/tracking_cmd", 1);
    status_ =
        create_publisher<std_msgs::msg::String>("control/tracking_status", 1);
    target_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "control/lookahead", 1);
    reference_view_ = create_publisher<kart_interfaces::msg::TrackingReference>(
        "control/tracking_reference", 1);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "control/tracking_markers", 1);
    mode_sub_ = create_subscription<kart_interfaces::msg::OperationModeState>(
        "operation_mode/state", 1,
        [this](kart_interfaces::msg::OperationModeState::ConstSharedPtr m) {
          mode_ = m->mode;
          mode_stamp_ = rclcpp::Time(m->header.stamp);
          mode_received_ = steady();
        });
    parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &values) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          try {
            for (const auto &p : values) if (tunable(p.get_name())) {
              if ((mode_ != kart_interfaces::msg::OperationModeState::STOP &&
                   mode_ != kart_interfaces::msg::OperationModeState::AUTO &&
                   mode_ != kart_interfaces::msg::OperationModeState::MANUAL) ||
                  !fresh(mode_stamp_, mode_received_, 0.3))
                throw std::invalid_argument("Fresh AUTO/MANUAL/STOP is required for tracking tuning");
              if (!std::isfinite(p.as_double()) || p.as_double() <= 0)
                throw std::invalid_argument("Tracking parameters must be finite and positive");
            }
          } catch (const std::exception &error) {
            result.successful = false;
            result.reason = error.what();
          }
          return result;
        });
    line_sub_ = create_subscription<kart_interfaces::msg::ReferenceLine>(
        "planning/reference_line", rclcpp::QoS(1).transient_local(),
        [this](kart_interfaces::msg::ReferenceLine::ConstSharedPtr m) {
          line_.clear();
          line_received_ = steady();
          line_stamp_ = rclcpp::Time(m->header.stamp);
          if (m->header.frame_id != map_ ||
              m->points.size() != m->speeds.size() || m->points.size() > 100000)
            return;
          for (size_t i = 0; i < m->points.size(); ++i) {
            if (!std::isfinite(m->points[i].z) ||
                std::abs(m->points[i].z) > 1e-6) {
              line_.clear();
              return;
            }
            line_.push_back({m->points[i].x, m->points[i].y, m->speeds[i]});
          }
          closed_ = m->closed;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "visual_slam/tracking/odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          pose_stamp_ = rclcpp::Time(m->header.stamp);
          pose_received_ = steady();
        });
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "localization/vslam/diagnostics", 10,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {
          bool seen = false, localized = false, tracking = false;
          for (const auto &s : m->status)
            if (s.hardware_id == "visual_slam") {
              seen = true;
              for (const auto &kv : s.values) {
                if (kv.key == "localized_in_exist_map")
                  localized = kv.value == "Yes";
                if (kv.key == "vo_status")
                  tracking = kv.value == "OK";
              }
            }
          if (seen) {
            localized_ = localized && tracking;
            diag_received_ = steady();
            diag_stamp_ = rclcpp::Time(m->header.stamp);
          }
        });
    if (get_parameter("use_sim_time").as_bool())
      timer_ = rclcpp::create_timer(this, get_clock(),
          rclcpp::Duration::from_seconds(1 / rate), [this] { tick(); });
    else
      timer_ = create_wall_timer(std::chrono::duration<double>(1 / rate),
          [this] { tick(); });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (steady() - pose_received_ > receive_limit(pose_timeout_, wall_timeout_) ||
          steady() - line_received_ > receive_limit(line_timeout_, wall_timeout_) ||
          steady() - diag_received_ > receive_limit(localization_timeout_, wall_timeout_))
        publish({}, "receive_timeout");
    });
  }

private:
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = !tunable(name);
    return declare_parameter<T>(name, value, d);
  }
  static bool tunable(const std::string &name) {
    return name == "lookahead_m" || name == "max_speed_mps" ||
           name == "max_cross_track_m" || name == "goal_tolerance_m" ||
           name == "deceleration_mps2";
  }
  static double steady() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  bool fresh(rclcpp::Time stamp, double received, double limit) {
    double age = (now() - stamp).seconds();
    return stamp.nanoseconds() > 0 && age >= -0.05 && age <= limit &&
           steady() - received <= receive_limit(limit, wall_timeout_);
  }
  void publish(const Result &r, const std::string &reason, Pose pose = {}) {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_;
    cmd.drive.steering_angle = r.valid ? r.steering : 0;
    cmd.drive.speed = r.valid ? r.speed : 0;
    command_->publish(cmd);
    std_msgs::msg::String status;
    status.data = reason;
    status_->publish(status);
    kart_interfaces::msg::TrackingReference view;
    view.header.frame_id = map_;
    view.header.stamp = r.valid ? pose_stamp_ : now();
    view.valid = r.valid;
    view.lookahead_m = c_.lookahead;
    if (r.valid) {
      view.vehicle_position.x = pose.x;
      view.vehicle_position.y = pose.y;
      view.target.x = r.target_x;
      view.target.y = r.target_y;
      view.target_distance_m = std::hypot(r.target_x - pose.x, r.target_y - pose.y);
    }
    reference_view_->publish(view);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header = view.header;
    marker.ns = "pure_pursuit";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    if (r.valid) {
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.lifetime.nanosec = 500000000;
      marker.pose.orientation.w = 1;
      marker.color.r = 1.0f;
      marker.color.g = 0.72f;
      marker.color.b = 0.12f;
      marker.color.a = 1;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.pose.position = view.target;
      marker.pose.position.z = 0.06;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.12;
      markers.markers.push_back(marker);
      marker.id = 1;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.pose.position = geometry_msgs::msg::Point();
      marker.points = {view.vehicle_position, view.target};
      marker.scale.x = 0.025;
      marker.scale.y = 0.08;
      marker.scale.z = 0.10;
      markers.markers.push_back(marker);
      marker.id = 2;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.points.clear();
      marker.pose.position = view.target;
      marker.pose.position.z = 0.25;
      marker.scale.z = 0.12;
      char label[128];
      std::snprintf(label, sizeof(label), "L(path)=%.2f m / d=%.2f m", c_.lookahead, view.target_distance_m);
      marker.text = label;
      markers.markers.push_back(marker);
    }
    markers_->publish(markers);
    if (r.valid) {
      geometry_msgs::msg::PointStamped p;
      p.header = cmd.header;
      p.header.frame_id = map_;
      p.point.x = r.target_x;
      p.point.y = r.target_y;
      target_->publish(p);
    }
  }
  bool inputs_ready() {
    if (!fresh(line_stamp_, line_received_, line_timeout_) || line_.empty()) {
      publish({}, "reference_unavailable");
      return false;
    }
    if (!fresh(pose_stamp_, pose_received_, pose_timeout_)) {
      publish({}, "pose_stale");
      return false;
    }
    if (!localized_ ||
        !fresh(diag_stamp_, diag_received_, localization_timeout_)) {
      publish({}, "not_localized");
      return false;
    }
    return true;
  }
  void tick() {
    // Read after the parameter transaction commits; never mutate control state
    // from a validation callback that a later validator could still reject.
    c_.lookahead = get_parameter("lookahead_m").as_double();
    c_.max_speed = get_parameter("max_speed_mps").as_double();
    c_.max_error = get_parameter("max_cross_track_m").as_double();
    c_.goal_tolerance = get_parameter("goal_tolerance_m").as_double();
    c_.deceleration = get_parameter("deceleration_mps2").as_double();
    if (c_.wheelbase <= 0 || c_.steering_limit <= 0) {
      publish({}, "vehicle_geometry_unconfigured");
      return;
    }
    if (!inputs_ready()) return;
    try {
      // Use the pose measurement time, never the static line publication time.
      // Odometry can reach this node before TF for the same stamp. Briefly wait
      // for that exact TF; never substitute a latest or stale transform.
      // A paused sim clock must not block the steady-time watchdog in this group.
      const double wait = get_parameter("use_sim_time").as_bool() ? 0.0 : tf_wait_;
      auto t = buffer_->lookupTransform(map_, frame_, pose_stamp_,
                                        rclcpp::Duration::from_seconds(wait));
      if (!inputs_ready()) return; // Waiting must not rejuvenate expired inputs.
      const auto &q = t.transform.rotation;
      double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(norm) || std::abs(norm - 1) > 0.01) {
        publish({}, "invalid_rotation");
        return;
      }
      double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z));
      auto result = pursue(
          line_, closed_,
          {t.transform.translation.x, t.transform.translation.y, yaw}, c_);
      publish(result, result.valid ? (result.speed > 0 ? "tracking"
                                                       : "zero_speed_target")
                                   : "invalid_geometry_or_off_path",
              {t.transform.translation.x, t.transform.translation.y, yaw});
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Tracking TF unavailable: %s <- %s at %.9f (pose age %.6f s): %s",
          map_.c_str(), frame_.c_str(), pose_stamp_.seconds(),
          (now() - pose_stamp_).seconds(), error.what());
      publish({}, "tf_unavailable");
    }
  }
  uint8_t mode_{0};
  double mode_received_{0};
  rclcpp::Time mode_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<kart_interfaces::msg::OperationModeState>::SharedPtr mode_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  Settings c_;
  std::string frame_, map_;
  bool closed_{false}, localized_{false};
  double pose_timeout_, line_timeout_, localization_timeout_, wall_timeout_, tf_wait_;
  double pose_received_{0}, line_received_{0}, diag_received_{0};
  rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME}, line_stamp_{0, 0, RCL_ROS_TIME},
      diag_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<Point> line_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<kart_interfaces::msg::ReferenceLine>::SharedPtr
      line_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diag_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      command_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_;
  rclcpp::Publisher<kart_interfaces::msg::TrackingReference>::SharedPtr reference_view_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_, watchdog_;
};
} // namespace kart_control
RCLCPP_COMPONENTS_REGISTER_NODE(kart_control::PurePursuitNode)
