#include "kart_e2e/latency_tracker.hpp"
#include "kart_interfaces/msg/control_command.hpp"
#include "kart_interfaces/msg/operation_mode_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>

namespace kart_e2e {
class LatencyProbeNode : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  using Command = kart_interfaces::msg::ControlCommand;
  using Mode = kart_interfaces::msg::OperationModeState;
  static int64_t ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
  static int64_t stamp(const std_msgs::msg::Header &h) { return int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec; }
  template <typename T> T setting(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
 public:
  explicit LatencyProbeNode(const rclcpp::NodeOptions &options) : Node("e2e_latency_probe", options) {
    const auto file = setting<std::string>("csv_path", "");
    warmup_s_ = setting<double>("warmup_s", 2.0);
    timeout_s_ = setting<double>("startup_timeout_s", 120.0);
    const auto maximum = setting<int>("max_pending", 4096);
    ready_topics_ = setting<std::vector<std::string>>("ready_topics", {});
    ready_nodes_ = setting<std::vector<std::string>>("ready_nodes", {});
    const auto service = setting<std::string>("resume_service", "/e2e_latency/rosbag2_player/resume");
    if (file.empty() || std::filesystem::exists(file) || maximum < 1 || maximum > 1000000 ||
        !std::isfinite(warmup_s_) || warmup_s_ < 0 || !std::isfinite(timeout_s_) || timeout_s_ <= 0 ||
        ready_topics_.empty() || ready_topics_.size() != ready_nodes_.size())
      throw std::invalid_argument("Invalid latency probe configuration or existing CSV");
    tracker_ = std::make_unique<LatencyTracker>(maximum);
    csv_.open(file);
    if (!csv_) throw std::runtime_error("Cannot create latency CSV");
    csv_ << "event,elapsed_s,source_stamp_ns,latency_ms,first,warmup,steering,throttle,brake,reverse,detail\n" << std::setprecision(12);
    start_ = ns();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("input/image", rclcpp::QoS(1));
    mode_pub_ = create_publisher<Mode>("operation_mode/state", rclcpp::QoS(1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("replay/image", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::UniquePtr msg) {
        const auto received = ns();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const bool valid = tracker_->input(stamp(msg->header), received, !auto_mode_);
          row(valid ? "input" : "ambiguous_input", received, stamp(msg->header));
        }
        image_pub_->publish(std::move(msg)); // Preserve header; intra-process handoff avoids copying image pixels.
      });
    decoded_sub_ = create_subscription<std_msgs::msg::Header>("e2e/decoded_frame", rclcpp::QoS(10),
      [this](std_msgs::msg::Header::ConstSharedPtr msg) { observe("decoded", *msg, nullptr); });
    command_sub_ = create_subscription<Command>("e2e/control_cmd", rclcpp::QoS(10),
      [this](Command::ConstSharedPtr msg) { observe("command", msg->header, msg.get()); });
    status_sub_ = create_subscription<std_msgs::msg::String>("e2e/status", rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        row("status", ns(), 0, {}, nullptr, msg->data);
      });
    resume_ = create_client<rosbag2_interfaces::srv::Resume>(service);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Latency CSV: %s; STOP warmup -> synthetic AUTO; no vehicle output", file.c_str());
  }
  ~LatencyProbeNode() override { csv_.flush(); }
 private:
  void row(const std::string &event, int64_t when, int64_t source,
           std::optional<LatencyTracker::Sample> sample = {}, const Command *cmd = nullptr,
           std::string detail = "") {
    // Only fixed status labels are written, but quote text defensively for CSV readers.
    std::string quoted;
    for (char c : detail) { quoted += c; if (c == '"') quoted += '"'; }
    csv_ << event << ',' << double(when-start_)/1e9 << ',' << source << ',';
    if (sample) csv_ << sample->ms;
    csv_ << ',' << (sample ? sample->first : false) << ',' << (sample ? sample->warmup : !auto_mode_) << ',';
    if (cmd) csv_ << cmd->steering << ',' << cmd->throttle << ',' << cmd->brake << ',' << cmd->reverse;
    else csv_ << ",,,";
    csv_ << ",\"" << quoted << "\"\n";
  }
  void observe(const std::string &stage, const std_msgs::msg::Header &header, const Command *cmd) {
    const auto received = ns();
    std::lock_guard<std::mutex> lock(mutex_);
    auto sample = tracker_->observe(stamp(header), received, cmd != nullptr);
    row(sample ? stage : "unmatched_"+stage, received, stamp(header), sample, cmd);
    if (!cmd && sample && first_decoded_ == 0) first_decoded_ = received;
  }
  void tick() {
    const auto current = ns();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!resuming_) {
      bool ready = resume_->service_is_ready();
      for (size_t i=0; ready && i<ready_topics_.size(); ++i) {
        bool found = false;
        for (const auto &info : get_subscriptions_info_by_topic(ready_topics_[i]))
          if (info.node_name() == ready_nodes_[i] && info.node_namespace() == get_namespace()) found = true;
        ready = found;
      }
      if (ready) {
        resuming_ = true;
        resume_->async_send_request(std::make_shared<rosbag2_interfaces::srv::Resume::Request>(),
          [this](rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedFuture future) {
            try { (void)future.get(); RCLCPP_INFO(get_logger(), "All pipeline subscribers ready; playback resumed"); }
            catch (const std::exception &e) { RCLCPP_ERROR(get_logger(), "%s", e.what()); rclcpp::shutdown(); }
          });
      }
    }
    if (!first_decoded_ && double(current-start_)/1e9 > timeout_s_) {
      row("startup_failed", current, 0, {}, nullptr, "no_decoded_frame_before_timeout");
      csv_.flush();
      RCLCPP_ERROR(get_logger(), "No decoded frame before startup timeout; check engine, topics and CUDA errors");
      rclcpp::shutdown();
      return;
    }
    if (!auto_mode_ && first_decoded_ && double(current-first_decoded_)/1e9 >= warmup_s_) {
      auto_mode_ = true;
      row("measurement_start", current, 0);
      RCLCPP_INFO(get_logger(), "Warmup complete; isolated decoder AUTO enabled");
    }
    Mode mode;
    mode.header.stamp = now();
    mode.mode = auto_mode_ ? Mode::AUTO : Mode::STOP;
    mode.source = "offline_latency_probe";
    mode_pub_->publish(mode);
    if (current-last_flush_ > 500000000LL) { csv_.flush(); last_flush_ = current; }
  }
  int64_t start_{0}, first_decoded_{0}, last_flush_{0};
  double warmup_s_, timeout_s_;
  bool resuming_{false}, auto_mode_{false};
  std::vector<std::string> ready_topics_, ready_nodes_;
  std::unique_ptr<LatencyTracker> tracker_;
  std::mutex mutex_;
  std::ofstream csv_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<Mode>::SharedPtr mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr decoded_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr resume_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace kart_e2e
RCLCPP_COMPONENTS_REGISTER_NODE(kart_e2e::LatencyProbeNode)
