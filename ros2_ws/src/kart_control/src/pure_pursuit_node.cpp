#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "kart_control/pure_pursuit.hpp"
#include "kart_interfaces/msg/reference_line.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
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
          param("lookahead_m", 0.8),       param("max_speed_mps", 1.0),
          param("max_cross_track_m", 1.0), param("goal_tolerance_m", 0.1),
          param("deceleration_mps2", 1.0)};
    pose_timeout_ = param("pose_timeout_s", 0.2);
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
    if (!std::isfinite(c_.wheelbase) || !std::isfinite(c_.steering_limit) ||
        c_.wheelbase < 0 || c_.steering_limit < 0 || c_.steering_limit >= 1.57)
      throw std::invalid_argument("Invalid vehicle geometry");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    command_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "control/tracking_cmd", 1);
    status_ =
        create_publisher<std_msgs::msg::String>("control/tracking_status", 1);
    target_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "control/lookahead", 1);
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
    timer_ = create_wall_timer(std::chrono::duration<double>(1 / rate),
                               [this] { tick(); });
  }

private:
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    return declare_parameter<T>(name, value, d);
  }
  static double steady() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  bool fresh(rclcpp::Time stamp, double received, double limit) {
    double age = (now() - stamp).seconds();
    return stamp.nanoseconds() > 0 && age >= -0.05 && age <= limit &&
           steady() - received <= limit;
  }
  void publish(const Result &r, const std::string &reason) {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_;
    cmd.drive.steering_angle = r.valid ? r.steering : 0;
    cmd.drive.speed = r.valid ? r.speed : 0;
    command_->publish(cmd);
    std_msgs::msg::String status;
    status.data = reason;
    status_->publish(status);
    if (r.valid) {
      geometry_msgs::msg::PointStamped p;
      p.header = cmd.header;
      p.header.frame_id = map_;
      p.point.x = r.target_x;
      p.point.y = r.target_y;
      target_->publish(p);
    }
  }
  void tick() {
    if (c_.wheelbase <= 0 || c_.steering_limit <= 0) {
      publish({}, "vehicle_geometry_unconfigured");
      return;
    }
    if (!fresh(line_stamp_, line_received_, line_timeout_) || line_.empty()) {
      publish({}, "reference_unavailable");
      return;
    }
    if (!fresh(pose_stamp_, pose_received_, pose_timeout_)) {
      publish({}, "pose_stale");
      return;
    }
    if (!localized_ ||
        !fresh(diag_stamp_, diag_received_, localization_timeout_)) {
      publish({}, "not_localized");
      return;
    }
    try {
      // Use the pose measurement time, never the static line publication time.
      auto t = buffer_->lookupTransform(map_, frame_, pose_stamp_,
                                        rclcpp::Duration::from_seconds(0));
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
                                   : "invalid_geometry_or_off_path");
    } catch (const tf2::TransformException &) {
      publish({}, "tf_unavailable");
    }
  }
  Settings c_;
  std::string frame_, map_;
  bool closed_{false}, localized_{false};
  double pose_timeout_, line_timeout_, localization_timeout_;
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
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace kart_control
RCLCPP_COMPONENTS_REGISTER_NODE(kart_control::PurePursuitNode)
