#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <stdexcept>
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <vector>
#include <array>
#include "visualization_msgs/msg/image_marker.hpp"

#include "ByteTrack/BYTETracker.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace kart_opponent_tracker
{
using Detections = vision_msgs::msg::Detection2DArray;

class OpponentTracker : public rclcpp::Node
{
public:
  explicit OpponentTracker(const rclcpp::NodeOptions & options)
  : Node("opponent_tracker", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    low_ = declare_parameter("low_score", 0.1, fixed);
    association_ = declare_parameter("association_score", 0.5, fixed);
    new_track_ = declare_parameter("new_track_score", 0.6, fixed);
    match_ = declare_parameter("match_threshold", 0.8, fixed);
    buffer_ = declare_parameter("lost_frames", 30, fixed);
    gap_ = declare_parameter("reset_gap_s", 0.5, fixed);
    if (!(0 <= low_ && low_ <= association_ && association_ <= new_track_ && new_track_ <= 1 &&
      0 < match_ && match_ <= 1 && buffer_ > 0 && std::isfinite(gap_) && gap_ > 0)) {
      throw std::invalid_argument("Invalid ByteTrack score thresholds, buffer or reset gap");
    }
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    output_ = create_publisher<Detections>("tracks", qos);
    boxes_ = create_publisher<visualization_msgs::msg::ImageMarker>("detection_boxes", qos);
    input_ = create_subscription<Detections>("detections", qos,
      [this](Detections::ConstSharedPtr msg) { update(*msg); });
  }

private:
  void update(const Detections & msg)
  {
    const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (tracker_ && frame_ == msg.header.frame_id && stamp == last_stamp_) {
      return;
    }
    if (!tracker_ || frame_ != msg.header.frame_id || stamp < last_stamp_ ||
      (stamp - last_stamp_) * 1e-9 > gap_)
    {
      tracker_ = std::make_unique<byte_track::BYTETracker>(30, buffer_, association_, new_track_, match_);
      ++generation_;
    }
    last_stamp_ = stamp;
    if (boxes_->get_subscription_count() + boxes_->get_intra_process_subscription_count() > 0) {
      publish_boxes(msg);
    }
    frame_ = msg.header.frame_id;
    std::vector<byte_track::Object> objects;
    for (const auto & detection : msg.detections) {
      const auto & box = detection.bbox;
      const float w = box.size_x, h = box.size_y;
      const float x = box.center.position.x - w / 2, y = box.center.position.y - h / 2;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) ||
        !std::isfinite(h) || w <= 0 || h <= 0) {continue;}
      for (const auto & result : detection.results) {
        const auto & hyp = result.hypothesis;
        if (hyp.class_id == "0" && hyp.score >= low_ && hyp.score <= 1.0) {
          objects.emplace_back(byte_track::Rect<float>(x, y, w, h), 0, hyp.score);
          break;
        }
      }
    }
    Detections tracks;
    tracks.header = msg.header;
    for (const auto & track : tracker_->update(objects)) {
      auto & d = tracks.detections.emplace_back();
      d.header = msg.header;
      d.id = std::to_string(generation_) + "-" + std::to_string(track->getTrackId());
      const auto & rect = track->getRect();
      d.bbox.center.position.x = rect.x() + rect.width() / 2;
      d.bbox.center.position.y = rect.y() + rect.height() / 2;
      d.bbox.size_x = rect.width();
      d.bbox.size_y = rect.height();
      auto & hypothesis = d.results.emplace_back().hypothesis;
      hypothesis.class_id = "0";
      hypothesis.score = track->getScore();
    }
    output_->publish(tracks);
  }

  void publish_boxes(const Detections & msg)
  {
    visualization_msgs::msg::ImageMarker marker;
    marker.header = msg.header;
    marker.ns = "opponent_detections";
    marker.type = marker.LINE_LIST;
    marker.scale = 2.0;
    marker.outline_color.g = 1.0;
    marker.outline_color.a = 1.0;
    for (const auto & detection : msg.detections) {
      const auto & box = detection.bbox;
      const auto x = box.center.position.x, y = box.center.position.y;
      const auto w = box.size_x / 2, h = box.size_y / 2;
      if (!(w > 0 && h > 0 && std::isfinite(x + y + w + h))) {continue;}
      std::array<geometry_msgs::msg::Point, 4> corners;
      corners[0].x = corners[3].x = x - w;
      corners[1].x = corners[2].x = x + w;
      corners[0].y = corners[1].y = y - h;
      corners[2].y = corners[3].y = y + h;
      for (size_t i = 0; i < corners.size(); ++i) {
        marker.points.push_back(corners[i]);
        marker.points.push_back(corners[(i + 1) % corners.size()]);
      }
    }
    boxes_->publish(marker);
  }

  rclcpp::Publisher<visualization_msgs::msg::ImageMarker>::SharedPtr boxes_;
  std::unique_ptr<byte_track::BYTETracker> tracker_;
  double low_, association_, new_track_, match_, gap_;
  int buffer_;
  int64_t last_stamp_{0};
  uint64_t generation_{0};
  std::string frame_;
  rclcpp::Subscription<Detections>::SharedPtr input_;
  rclcpp::Publisher<Detections>::SharedPtr output_;
};
}  // namespace kart_opponent_tracker
RCLCPP_COMPONENTS_REGISTER_NODE(kart_opponent_tracker::OpponentTracker)
