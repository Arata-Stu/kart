#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "kart_control/speed_pid.hpp"
#include "kart_interfaces/msg/control_command.hpp"
#include "kart_interfaces/msg/operation_mode_request.hpp"
#include "kart_interfaces/msg/operation_mode_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace kart_control {
class SpeedControllerNode : public rclcpp::Node {
  using Mode = kart_interfaces::msg::OperationModeState;
  using Command = kart_interfaces::msg::ControlCommand;

public:
  explicit SpeedControllerNode(const rclcpp::NodeOptions &options)
      : Node("speed_controller", options) {
    frame_ = fixed<std::string>("tracking_frame", "rear_axle");
    steering_limit_ = fixed("max_steering_rad", 0.0);
    timeout_ = fixed("input_timeout_s", 0.15);
    mode_timeout_ = fixed("mode_timeout_s", 0.3);
    max_speed_ = fixed("max_target_speed_mps", 1.0);
    rate_ = fixed("control_rate_hz", 50.0);
    for (double v : {timeout_, mode_timeout_, max_speed_, rate_})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument("Positive parameters required");
    if (frame_.empty() || frame_[0] == '/' || !std::isfinite(steering_limit_) ||
        steering_limit_ < 0 || steering_limit_ >= 1.57)
      throw std::invalid_argument("Invalid steering/frame calibration");
    for (const auto &item :
         std::vector<std::pair<std::string, double>>{{"kp", 0},
                                                     {"ki", 0},
                                                     {"kd", 0},
                                                     {"kff", 0},
                                                     {"integral_limit", 0.2},
                                                     {"derivative_tau_s", 0.1},
                                                     {"throttle_limit", 0.2},
                                                     {"brake_limit", 0}})
      declare_parameter(item.first, item.second);
    settings_ = read_settings();
    validate(settings_);
    parameter_callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &values) {
          auto candidate = read_settings();
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          try {
            for (const auto &p : values) {
              if (p.get_name() == "kp")
                candidate.kp = p.as_double();
              else if (p.get_name() == "ki")
                candidate.ki = p.as_double();
              else if (p.get_name() == "kd")
                candidate.kd = p.as_double();
              else if (p.get_name() == "kff")
                candidate.kff = p.as_double();
              else if (p.get_name() == "integral_limit")
                candidate.integral_limit = p.as_double();
              else if (p.get_name() == "derivative_tau_s")
                candidate.derivative_tau = p.as_double();
              else if (p.get_name() == "throttle_limit")
                candidate.throttle_limit = p.as_double();
              else if (p.get_name() == "brake_limit")
                candidate.brake_limit = p.as_double();
            }
            validate(candidate);
          } catch (const std::exception &e) {
            result.successful = false;
            result.reason = e.what();
          }
          return result;
        });
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    pub_ = create_publisher<Command>("auto/control_cmd", 1);
    stop_ = create_publisher<kart_interfaces::msg::OperationModeRequest>(
        "operation_mode/request", 10);
    diagnostic_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "control/speed_diagnostics", 1);
    target_sub_ =
        create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "control/tracking_cmd", 1,
            [this](
                ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
              target_ = *m;
              target_received_ = steady();
            });
    status_sub_ = create_subscription<std_msgs::msg::String>(
        "control/tracking_status", 1,
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          tracking_ok_ =
              m->data == "tracking" || m->data == "zero_speed_target";
          status_received_ = steady();
        });
    mode_sub_ = create_subscription<Mode>("operation_mode/state", 1,
                                          [this](Mode::ConstSharedPtr m) {
                                            mode_ = *m;
                                            mode_received_ = steady();
                                          });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "visual_slam/tracking/odometry", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { measure(*m); });
    previous_tick_ = steady();
    timer_ = create_wall_timer(std::chrono::duration<double>(1 / rate_),
                               [this] { tick(); });
  }

private:
  template <class T> T fixed(const std::string &key, T value) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    return declare_parameter<T>(key, value, d);
  }
  static double steady() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  PidSettings read_settings() {
    return {get_parameter("kp").as_double(),
            get_parameter("ki").as_double(),
            get_parameter("kd").as_double(),
            get_parameter("kff").as_double(),
            get_parameter("integral_limit").as_double(),
            get_parameter("derivative_tau_s").as_double(),
            get_parameter("throttle_limit").as_double(),
            get_parameter("brake_limit").as_double()};
  }
  bool fresh(const builtin_interfaces::msg::Time &stamp, double received,
             double limit) {
    double age = (now() - rclcpp::Time(stamp)).seconds();
    return rclcpp::Time(stamp).nanoseconds() > 0 && age >= -0.05 &&
           age <= limit && steady() - received <= limit;
  }
  void measure(const nav_msgs::msg::Odometry &m) {
    measurement_ok_ = false;
    if (m.child_frame_id.empty())
      return;
    try {
      auto t = buffer_->lookupTransform(frame_, m.child_frame_id,
                                        rclcpp::Time(m.header.stamp),
                                        rclcpp::Duration::from_seconds(0));
      const auto &q = t.transform.rotation;
      tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
      if (!std::isfinite(rotation.length2()) ||
          std::abs(rotation.length2() - 1) > 0.01)
        return;
      const auto &twist = m.twist.twist;
      tf2::Vector3 linear(twist.linear.x, twist.linear.y, twist.linear.z),
          angular(twist.angular.x, twist.angular.y, twist.angular.z);
      auto velocity = tf2::quatRotate(rotation, linear);
      auto omega = tf2::quatRotate(rotation, angular);
      // p points from rear axle to odometry child origin. v_child = v_axle +
      // omega x p.
      auto p =
          tf2::Vector3(t.transform.translation.x, t.transform.translation.y,
                       t.transform.translation.z);
      measured_ = (velocity - omega.cross(p)).x();
      if (!std::isfinite(measured_))
        return;
      measured_stamp_ = m.header.stamp;
      measured_received_ = steady();
      measurement_ok_ = true;
    } catch (const tf2::TransformException &) {
    }
  }
  void output(const std::string &state, PidOutput result = {},
              double steering = 0) {
    Command command;
    command.header.stamp = now();
    command.header.frame_id = frame_;
    if (state == "active") {
      // Preserve the oldest upstream sample time instead of rejuvenating stale
      // commands.
      auto stamp = std::min(rclcpp::Time(target_.header.stamp),
                            rclcpp::Time(measured_stamp_));
      command.header.stamp = stamp;
      command.steering = steering;
      command.throttle = result.throttle;
      command.brake = result.brake;
    }
    pub_->publish(command);
    if (steady() - last_diagnostic_ < 0.1)
      return;
    last_diagnostic_ = steady();
    diagnostic_msgs::msg::DiagnosticArray d;
    d.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "speed_controller";
    s.hardware_id = "speed_controller";
    s.message = state;
    s.level = state == "active" || state == "inactive" ? 0 : 1;
    for (const auto &item : std::vector<std::pair<std::string, double>>{
             {"target_mps", target_.drive.speed},
             {"measured_mps", measured_},
             {"error_mps", target_.drive.speed - measured_},
             {"p", result.p},
             {"i", result.i},
             {"d", result.d},
             {"ff", result.ff},
             {"throttle", result.throttle},
             {"brake", result.brake}}) {
      diagnostic_msgs::msg::KeyValue v;
      v.key = item.first;
      v.value = std::to_string(item.second);
      s.values.push_back(v);
    }
    d.status.push_back(s);
    diagnostic_->publish(d);
  }
  void fault(const std::string &why) {
    pid_.reset();
    latched_ = true;
    if (steady() - last_stop_ >= 0.1) {
      kart_interfaces::msg::OperationModeRequest r;
      r.header.stamp = now();
      r.mode = Mode::STOP;
      r.source = "speed_controller:" + why;
      stop_->publish(r);
      last_stop_ = steady();
    }
    output(why);
  }
  void tick() {
    double time = steady(), dt = time - previous_tick_;
    previous_tick_ = time;
    auto updated = read_settings();
    if (!(updated == settings_)) {
      settings_ = updated;
      pid_.reset();
      RCLCPP_INFO(get_logger(),
                  "PID parameters changed; integrator and derivative reset");
    }
    const bool mode_fresh =
        fresh(mode_.header.stamp, mode_received_, mode_timeout_);
    if (mode_fresh && (mode_.mode == Mode::STOP || mode_.mode == Mode::MANUAL ||
                       mode_.mode == Mode::PROPO)) {
      pid_.reset();
      latched_ = false;
      was_auto_ = false;
      output("inactive");
      return;
    }
    if (!mode_fresh) {
      if (was_auto_)
        fault("mode_stale");
      else {
        pid_.reset();
        output("waiting_for_mode");
      }
      return;
    }
    if (mode_.mode != Mode::AUTO) {
      fault("invalid_mode");
      return;
    }
    if (!was_auto_) {
      auto_ready_at_ = time + 0.1;
      pid_.reset();
    }
    was_auto_ = true;
    if (latched_) {
      fault("fault_latched");
      return;
    }
    if (steering_limit_ <= 0) {
      fault("steering_uncalibrated");
      return;
    }
    if (!fresh(target_.header.stamp, target_received_, timeout_) ||
        target_.header.frame_id != frame_ || !tracking_ok_ ||
        time - status_received_ > timeout_) {
      fault("tracking_unavailable");
      return;
    }
    if (!measurement_ok_ ||
        !fresh(measured_stamp_, measured_received_, timeout_)) {
      fault("speed_unavailable");
      return;
    }
    double desired = target_.drive.speed, angle = target_.drive.steering_angle;
    if (!std::isfinite(desired) || !std::isfinite(angle) || desired < 0 ||
        desired > max_speed_) {
      fault("invalid_target");
      return;
    }
    if (dt <= 0 || dt > 0.2) {
      fault("control_deadline");
      return;
    }
    if (time < auto_ready_at_) {
      output("arming");
      return;
    }
    try {
      if (desired == 0)
        pid_.reset();
      auto result = pid_.step(desired, measured_, dt, settings_);
      if (desired == 0) {
        result.throttle = 0;
        pid_.reset();
      }
      output("active", result, std::clamp(angle / steering_limit_, -1.0, 1.0));
    } catch (const std::exception &) {
      fault("pid_invalid");
    }
  }
  std::string frame_;
  double steering_limit_, timeout_, mode_timeout_, max_speed_, rate_;
  SpeedPid pid_;
  PidSettings settings_;
  bool tracking_ok_{false}, measurement_ok_{false}, latched_{false},
      was_auto_{false};
  double target_received_{0}, status_received_{0}, mode_received_{0},
      measured_received_{0}, previous_tick_{0}, last_stop_{0},
      last_diagnostic_{0}, measured_{0}, auto_ready_at_{0};
  builtin_interfaces::msg::Time measured_stamp_;
  Mode mode_;
  ackermann_msgs::msg::AckermannDriveStamped target_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<Command>::SharedPtr pub_;
  rclcpp::Publisher<kart_interfaces::msg::OperationModeRequest>::SharedPtr
      stop_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace kart_control
RCLCPP_COMPONENTS_REGISTER_NODE(kart_control::SpeedControllerNode)
