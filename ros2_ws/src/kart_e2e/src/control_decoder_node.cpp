#include "cuda_buffer/cuda_buffer_api.hpp"
#include "isaac_ros_common/cuda_stream.hpp"
#include "isaac_ros_tensor_msgs/msg/tensor_list.hpp"
#include "kart_e2e/control_gate.hpp"
#include "kart_interfaces/msg/control_command.hpp"
#include "kart_interfaces/msg/operation_mode_request.hpp"
#include "kart_interfaces/msg/operation_mode_state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>
#include <mutex>

namespace kart_e2e {
class ControlDecoderNode : public rclcpp::Node {
  using TensorList = isaac_ros_tensor_msgs::msg::TensorList;
  using Mode = kart_interfaces::msg::OperationModeState;
  using Clock = std::chrono::steady_clock;
  template <typename T> T setting(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  static double seconds(const builtin_interfaces::msg::Time &t) {
    return double(t.sec) + double(t.nanosec) * 1e-9;
  }
  static double age(Clock::time_point t) {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }

public:
  explicit ControlDecoderNode(const rclcpp::NodeOptions &options)
      : Node("e2e_control_decoder", options) {
    const auto mode = setting<std::string>("output_mode", "steer_throttle");
    if (mode != "steer_only" && mode != "steer_throttle")
      throw std::invalid_argument("Invalid output_mode");
    count_ = mode == "steer_only" ? 1 : 2;
    tensor_name_ = setting<std::string>("output_tensor_name", "control");
    timeout_ = setting<double>("input_timeout_s", .15);
    mode_timeout_ = setting<double>("mode_timeout_s", .3);
    const auto fixed = setting<double>("fixed_throttle", 0.0);
    const auto maximum = setting<double>("max_throttle", .2);
    drive_ = setting<bool>("drive_enabled", false);
    const auto rate = setting<double>("control_rate_hz", 50.0);
    if (!std::isfinite(rate) || rate < 1 || rate > 100 || tensor_name_.empty())
      throw std::invalid_argument("Invalid control rate/tensor name");
    gate_ = std::make_unique<ControlGate>(count_ == 1, fixed, maximum, timeout_,
                                          mode_timeout_);
    stream_ = nvidia::isaac_ros::common::createCudaStream("E2EControlDecoder");
    output_ = create_publisher<kart_interfaces::msg::ControlCommand>(
        drive_ ? "auto/control_cmd" : "e2e/control_cmd", 1);
    stop_ = create_publisher<kart_interfaces::msg::OperationModeRequest>(
        "operation_mode/request", 10);
    status_ = create_publisher<std_msgs::msg::String>("e2e/status", 1);
    mode_sub_ = create_subscription<Mode>(
        "operation_mode/state", 1, [this](Mode::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (msg->mode != mode_.mode) {
            ++generation_;
            prediction_.clear();
            gate_->transition();
          }
          mode_ = *msg;
          mode_received_ = Clock::now();
        });
    // CUDA readback may wait for the producer. Keep the watchdog in another
    // group.
    tensor_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = tensor_group_;
    sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    sub_options.acceptable_buffer_backends = "any";
    tensor_sub_ = create_subscription<TensorList>(
        "e2e/tensor_output", rclcpp::QoS(1),
        [this](TensorList::ConstSharedPtr msg) { decode(msg); }, sub_options);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               [this]() { tick(); });
  }

private:
  void decode(const TensorList::ConstSharedPtr &msg) {
    const auto received = Clock::now();
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      generation = generation_;
    }
    std::vector<float> values;
    try {
      if (msg->names.size() != 1 || msg->tensors.size() != 1 ||
          msg->names[0] != tensor_name_)
        throw std::runtime_error("Expected one named control tensor");
      const auto &tensor = msg->tensors[0];
      if (tensor.dtype_code != 2 || tensor.dtype_bits != 32 ||
          tensor.dtype_lanes != 1 || tensor.shape.size() != 2 ||
          tensor.shape[0] != 1 || tensor.shape[1] != count_ ||
          !tensor.strides.empty() || tensor.byte_offset != 0 ||
          tensor.data.size() != static_cast<size_t>(count_) * sizeof(float))
        throw std::runtime_error(
            "Expected contiguous FP32 [1,N], N matching output_mode");
      values.resize(count_);
      auto handle =
          cuda_buffer_backend::from_input_buffer(tensor.data, *stream_);
      if (!handle.get_ptr())
        throw std::runtime_error("Null tensor buffer");
      const auto copied = cudaMemcpyAsync(values.data(), handle.get_ptr(),
                                          values.size() * sizeof(float),
                                          cudaMemcpyDeviceToHost, *stream_);
      const auto synchronized = cudaStreamSynchronize(*stream_);
      if (copied != cudaSuccess || synchronized != cudaSuccess)
        throw std::runtime_error("Tensor CUDA readback failed");
    } catch (const std::exception &error) {
      values.clear();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                           error.what());
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_)
      return;
    prediction_ = std::move(values);
    header_ = msg->header;
    received_ = received;
  }
  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto time = now();
    const auto sec = time.seconds();
    if (last_time_ >= 0 && sec < last_time_) {
      gate_->fault();
      ++generation_;
      prediction_.clear();
    }
    last_time_ = sec;
    if (age(received_) > timeout_)
      prediction_.clear();
    const auto mode_stamp = age(mode_received_) <= mode_timeout_
                                ? seconds(mode_.header.stamp)
                                : 0.0;
    const auto result = gate_->step(
        sec, mode_.mode, mode_stamp, prediction_, seconds(header_.stamp),
        drive_ && count_publishers(output_->get_topic_name()) > 1);
    kart_interfaces::msg::ControlCommand command;
    command.header.stamp = time;
    if (std::string(result.status) == "active")
      command.header = header_;
    command.steering = result.steering;
    command.throttle = result.throttle;
    output_->publish(command);
    if (age(reported_) >= .1) {
      std_msgs::msg::String status;
      status.data = result.status;
      status_->publish(status);
      if (drive_ && result.stop) {
        kart_interfaces::msg::OperationModeRequest request;
        request.header.stamp = time;
        request.mode = Mode::STOP;
        request.source = "e2e_control_decoder";
        stop_->publish(request);
      }
      reported_ = Clock::now();
    }
  }
  std::string tensor_name_;
  int count_;
  bool drive_;
  double timeout_, mode_timeout_, last_time_{-1};
  uint64_t generation_{0};
  std::mutex mutex_;
  Mode mode_;
  std_msgs::msg::Header header_;
  Clock::time_point received_{}, mode_received_{}, reported_{};
  std::vector<float> prediction_;
  std::unique_ptr<ControlGate> gate_;
  nvidia::isaac_ros::common::CudaStreamPtr stream_;
  rclcpp::CallbackGroup::SharedPtr tensor_group_;
  rclcpp::Subscription<TensorList>::SharedPtr tensor_sub_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Publisher<kart_interfaces::msg::ControlCommand>::SharedPtr output_;
  rclcpp::Publisher<kart_interfaces::msg::OperationModeRequest>::SharedPtr
      stop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace kart_e2e
RCLCPP_COMPONENTS_REGISTER_NODE(kart_e2e::ControlDecoderNode)
