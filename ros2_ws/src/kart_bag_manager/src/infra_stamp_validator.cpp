#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <kart_interfaces/msg/infra_stamp_result.hpp>

namespace kart_bag_manager
{
class InfraStampValidator : public rclcpp::Node
{
public:
  explicit InfraStampValidator(const rclcpp::NodeOptions & options)
  : Node("infra_stamp_validator", options)
  {
    const auto hz = declare_parameter("expected_hz", 90.0);
    if (!std::isfinite(hz) || hz <= 0) {
      throw std::invalid_argument("expected_hz must be finite and positive");
    }
    period_ns_ = 1e9 / hz;
    publisher_ = create_publisher<Result>("/validation/infra_stamps", 100);
    subscribe<sensor_msgs::msg::Image>(Result::LEFT_IMAGE, "/realsense/infra1/image_rect_raw");
    subscribe<sensor_msgs::msg::Image>(Result::RIGHT_IMAGE, "/realsense/infra2/image_rect_raw");
    subscribe<sensor_msgs::msg::CameraInfo>(Result::LEFT_INFO, "/realsense/infra1/camera_info");
    subscribe<sensor_msgs::msg::CameraInfo>(Result::RIGHT_INFO, "/realsense/infra2/camera_info");
  }

private:
  using Result = kart_interfaces::msg::InfraStampResult;

  template<typename Message>
  void subscribe(uint8_t stream, const char * topic)
  {
    subscriptions_[stream] = create_subscription<Message>(
      topic, rclcpp::SensorDataQoS(), [this, stream](typename Message::ConstSharedPtr msg) {
        observe(stream, rclcpp::Time(msg->header.stamp).nanoseconds());
      });
  }

  void observe(uint8_t stream, int64_t stamp)
  {
    Result result;
    result.stream = stream;
    result.stamp_ns = stamp;
    result.sequence = ++counts_[stream];
    result.interval_ns = result.sequence == 1 ? 0 : stamp - previous_[stream];
    previous_[stream] = stamp;
    if (result.interval_ns > period_ns_) {
      result.expected_frames_missing = std::llround(result.interval_ns / period_ns_) - 1;
    }
    if (seen_.find(stamp) == seen_.end()) {
      recent_.push_back(stamp);
      if (recent_.size() > 512) {
        seen_.erase(recent_.front());
        recent_.pop_front();
      }
    }
    result.seen_mask = (seen_[stamp] |= 1u << stream);
    publisher_->publish(result);
  }

  double period_ns_;
  std::array<uint64_t, 4> counts_{};
  std::array<int64_t, 4> previous_{};
  std::deque<int64_t> recent_;
  std::unordered_map<int64_t, uint8_t> seen_;
  std::array<rclcpp::SubscriptionBase::SharedPtr, 4> subscriptions_;
  rclcpp::Publisher<Result>::SharedPtr publisher_;
};
}  // namespace kart_bag_manager

RCLCPP_COMPONENTS_REGISTER_NODE(kart_bag_manager::InfraStampValidator)
