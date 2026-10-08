#include "kart_joy/mapping.hpp"

#include <libevdev/libevdev.h>
#include <yaml-cpp/yaml.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace kart_joy
{
namespace
{
void keys(const YAML::Node & node, const std::vector<std::string> & allowed)
{
  if (!node.IsMap()) {throw std::invalid_argument("expected YAML mapping");}
  for (const auto & entry : node) {
    const auto key = entry.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      throw std::invalid_argument("unknown profile key: " + key);
    }
  }
}
struct Profile
{
  int vendor{1356}, product{3302};
  std::string name, unique;
  std::vector<AxisBinding> axes;
  std::vector<ButtonBinding> buttons;
  static Profile load(const std::string & path)
  {
    if (std::filesystem::file_size(path) > 65536) {
      throw std::invalid_argument("profile exceeds 64 KiB");
    }
    const auto root = YAML::LoadFile(path);
    keys(root, {"version", "device", "axes", "buttons"});
    if (root["version"].as<int>() != 1) {throw std::invalid_argument("unsupported profile version");}
    keys(root["device"], {"vendor", "product", "name", "unique"});
    keys(root["axes"], axis_names);
    keys(root["buttons"], button_names);
    Profile p;
    p.vendor = root["device"]["vendor"].as<int>();
    p.product = root["device"]["product"].as<int>();
    p.name = root["device"]["name"].as<std::string>("");
    p.unique = root["device"]["unique"].as<std::string>("");
    if (p.vendor < 0 || p.vendor > 65535 || p.product < 0 || p.product > 65535) {
      throw std::invalid_argument("invalid vendor/product ID");
    }
    for (const auto & name : axis_names) {
      const auto node = root["axes"][name];
      keys(node, {"code", "mode", "negative", "center", "positive", "deadzone"});
      AxisBinding a;
      a.code = node["code"].as<int>();
      const auto mode = node["mode"].as<std::string>();
      if (mode != "bipolar" && mode != "unipolar") {throw std::invalid_argument("invalid axis mode");}
      a.unipolar = mode == "unipolar";
      a.negative = node["negative"].as<double>();
      a.center = node["center"].as<double>();
      a.positive = node["positive"].as<double>();
      a.deadzone = node["deadzone"].as<double>();
      a.validate();
      p.axes.push_back(a);
    }
    for (const auto & name : button_names) {
      const auto node = root["buttons"][name];
      keys(node, {"source", "code", "direction", "threshold"});
      ButtonBinding b;
      const auto source = node["source"].as<std::string>();
      if (source != "axis" && source != "button") {throw std::invalid_argument("invalid button source");}
      b.from_axis = source == "axis";
      b.code = node["code"].as<int>();
      b.direction = node["direction"].as<int>();
      b.threshold = node["threshold"].as<double>();
      b.validate();
      p.buttons.push_back(b);
    }
    return p;
  }
};
struct Device
{
  int fd{-1};
  libevdev * handle{nullptr};
  std::string path;
  ~Device() {if (handle) {libevdev_free(handle);} if (fd >= 0) {::close(fd);}}
};
std::string text(const char * value) {return value ? value : "";}
}  // namespace

class JoyNode : public rclcpp::Node
{
public:
  explicit JoyNode(const rclcpp::NodeOptions & options)
  : Node("kart_joy_node", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    profile_path_ = declare_parameter<std::string>("profile_path", "", fixed);
    input_dir_ = declare_parameter<std::string>("input_dir", "/dev/input", fixed);
    const double rate = declare_parameter<double>("publish_rate_hz", 100.0, fixed);
    const int scan_ms = declare_parameter<int>("scan_period_ms", 1000, fixed);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 1000.0 || scan_ms < 100) {
      throw std::invalid_argument("publish_rate_hz must be 1..1000; scan_period_ms >= 100");
    }
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(1).reliable());
    raw_pub_ = create_publisher<sensor_msgs::msg::Joy>("joy/raw", rclcpp::SensorDataQoS().keep_last(1));
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("joy/connected", rclcpp::QoS(1));
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("joy/ready", rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>("joy/status", rclcpp::QoS(1));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
    reload_ = create_service<std_srvs::srv::Trigger>("~/reload_profile",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!configuring_) {response->message = "enter configuration mode first"; return;}
        response->success = load_profile();
        response->message = response->success ? "profile loaded; output remains neutral" : error_;
      });
    configure_ = create_service<std_srvs::srv::SetBool>("~/configure",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        const auto mapped = map_state(device_ && !syncing_, raw_axes_, raw_keys_, profile_.axes, profile_.buttons);
        if (!request->data && (!device_ || !profile_valid_ || syncing_ || !bindings_available(device_->handle) ||
          std::any_of(mapped.axes.begin(), mapped.axes.end(), [](float v) {return std::abs(v) > 0.1F;}) ||
          std::any_of(mapped.buttons.begin(), mapped.buttons.end(), [](int v) {return v != 0;})))
        {
          response->message = "connect controller and release all controls before resuming";
          return;
        }
        configuring_ = request->data;
        response->success = true;
        response->message = configuring_ ? "configuration mode: output neutral" : "output resumed";
        publish();
        status();
      });
    load_profile();
    scan_timer_ = create_wall_timer(std::chrono::milliseconds(scan_ms), [this]() {scan();});
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {read_events(); publish();});
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {status();});
    scan();
  }

private:
  bool load_profile()
  {
    try {
      auto candidate = Profile::load(profile_path_);
      profile_ = std::move(candidate);
      profile_valid_ = true;
      error_.clear();
      disconnect();
      ++generation_;
      return true;
    } catch (const std::exception & e) {
      // Invalid reloads leave the last valid profile intact and stay neutral.
      error_ = e.what();
      RCLCPP_ERROR(get_logger(), "Profile: %s", error_.c_str());
      return false;
    }
  }

  void disconnect()
  {
    device_.reset();
    syncing_ = false;
    std::fill(raw_axes_.begin(), raw_axes_.end(), 0.0F);
    std::fill(raw_keys_.begin(), raw_keys_.end(), 0);
  }

  bool bindings_available(libevdev * dev) const
  {
    for (const auto & a : profile_.axes) {
      if (a.code >= 0 && !libevdev_has_event_code(dev, EV_ABS, a.code)) {return false;}
      if (a.code >= 0) {
        const auto * info = libevdev_get_abs_info(dev, a.code);
        if (!info || info->maximum <= info->minimum) {return false;}
      }
    }
    for (const auto & b : profile_.buttons) {
      if (b.code >= 0 && !libevdev_has_event_code(dev, b.from_axis ? EV_ABS : EV_KEY, b.code)) {return false;}
    }
    return true;
  }

  void scan()
  {
    if (device_ || !profile_valid_) {return;}
    std::vector<std::unique_ptr<Device>> matches;
    bool permission_denied = false;
    std::error_code ec;
    for (const auto & entry : std::filesystem::directory_iterator(input_dir_, ec)) {
      if (entry.path().filename().string().rfind("event", 0) != 0) {continue;}
      auto candidate = std::make_unique<Device>();
      candidate->path = entry.path().string();
      candidate->fd = ::open(candidate->path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (candidate->fd < 0) {permission_denied |= errno == EACCES; continue;}
      if (libevdev_new_from_fd(candidate->fd, &candidate->handle) < 0) {continue;}
      auto * dev = candidate->handle;
      if (libevdev_get_id_vendor(dev) != profile_.vendor ||
        libevdev_get_id_product(dev) != profile_.product ||
        (!profile_.name.empty() && text(libevdev_get_name(dev)).find(profile_.name) == std::string::npos) ||
        (!profile_.unique.empty() && text(libevdev_get_uniq(dev)) != profile_.unique) ||
        !libevdev_has_event_code(dev, EV_ABS, ABS_X) ||
        !libevdev_has_event_code(dev, EV_ABS, ABS_Y) ||
        !libevdev_has_event_code(dev, EV_KEY, BTN_GAMEPAD)) {continue;}
      matches.push_back(std::move(candidate));
    }
    if (matches.size() != 1) {
      error_ = matches.size() > 1 ? "multiple matching controllers; set device.unique" :
        permission_denied ? "no readable matching controller; check /dev/input permissions" :
        "waiting for controller";
      return;
    }
    device_ = std::move(matches.front());
    error_.clear();
    ++generation_;
    snapshot();
    RCLCPP_INFO(get_logger(), "Connected %s (%s)", text(libevdev_get_name(device_->handle)).c_str(), device_->path.c_str());
    status();
  }

  void snapshot()
  {
    for (int code = 0; code < kAxisCount; ++code) {
      const auto * info = libevdev_get_abs_info(device_->handle, code);
      raw_axes_[code] = info && info->maximum > info->minimum ?
        static_cast<float>(std::clamp(2.0 * (static_cast<double>(info->value) - info->minimum) /
        (static_cast<double>(info->maximum) - info->minimum) - 1.0, -1.0, 1.0)) : 0.0F;
    }
    for (int code = 0; code < kKeyCount; ++code) {
      raw_keys_[code] = libevdev_get_event_value(device_->handle, EV_KEY, code) != 0;
    }
  }

  void read_events()
  {
    if (!device_) {return;}
    // Bound callback work so a noisy device cannot starve a component executor.
    for (int count = 0; count < 512; ++count) {
      input_event event{};
      const int rc = libevdev_next_event(device_->handle,
        syncing_ ? LIBEVDEV_READ_FLAG_SYNC : LIBEVDEV_READ_FLAG_NORMAL, &event);
      if (rc == -EINTR) {continue;}
      if (rc == -EAGAIN) {
        if (syncing_) {syncing_ = false; snapshot(); continue;}
        return;
      }
      if (rc < 0) {
        error_ = "controller disconnected or read failed";
        disconnect();
        ++generation_;
        status();
        return;
      }
      if (rc == LIBEVDEV_READ_STATUS_SYNC) {syncing_ = true;}
      if (!syncing_ && event.type == EV_SYN && event.code == SYN_REPORT) {snapshot();}
    }
  }

  void publish()
  {
    sensor_msgs::msg::Joy raw;
    raw.header.stamp = now();
    raw.header.frame_id = "kart_joy_raw_" + std::to_string(generation_);
    raw.axes = raw_axes_;
    raw.buttons = raw_keys_;
    raw_pub_->publish(raw);
    const bool ready = device_ && !syncing_ && profile_valid_ && bindings_available(device_->handle);
    const auto mapped = map_state(ready && !configuring_, raw_axes_, raw_keys_, profile_.axes, profile_.buttons);
    sensor_msgs::msg::Joy msg;
    msg.header = raw.header;
    msg.header.frame_id = "kart_gamepad";
    msg.axes = mapped.axes;
    msg.buttons = mapped.buttons;
    joy_pub_->publish(msg);
    std_msgs::msg::Bool connected;
    connected.data = ready;
    connected_pub_->publish(connected);
    connected.data = ready && !configuring_;
    ready_pub_->publish(connected);
  }

  void status()
  {
    YAML::Node data;
    data["connected"] = static_cast<bool>(device_);
    data["configuring"] = configuring_;
    data["generation"] = generation_;
    data["profile_path"] = profile_path_;
    data["profile_valid"] = profile_valid_;
    data["node"] = get_fully_qualified_name();
    data["error"] = error_;
    data["axes"] = YAML::Node(YAML::NodeType::Sequence);
    data["buttons"] = YAML::Node(YAML::NodeType::Sequence);
    const bool available = device_ && bindings_available(device_->handle);
    data["ready"] = available && !syncing_ && profile_valid_ && !configuring_;
    if (device_) {
      auto * dev = device_->handle;
      data["device"]["name"] = text(libevdev_get_name(dev));
      data["device"]["path"] = device_->path;
      data["device"]["unique"] = text(libevdev_get_uniq(dev));
      data["device"]["vendor"] = libevdev_get_id_vendor(dev);
      data["device"]["product"] = libevdev_get_id_product(dev);
      if (!available) {data["error"] = "profile refers to unavailable inputs; output neutral";}
      for (const auto type : {EV_ABS, EV_KEY}) {
        for (int code = 0; code < (type == EV_ABS ? kAxisCount : kKeyCount); ++code) {
          if (!libevdev_has_event_code(dev, type, code)) {continue;}
          YAML::Node item;
          item["code"] = code;
          item["name"] = text(libevdev_event_code_get_name(type, code));
          data[type == EV_ABS ? "axes" : "buttons"].push_back(item);
        }
      }
    }
    std_msgs::msg::String status_msg;
    status_msg.data = YAML::Dump(data);
    status_pub_->publish(status_msg);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus entry;
    entry.name = std::string(get_fully_qualified_name()) + ": joystick";
    entry.hardware_id = device_ ? device_->path : "disconnected";
    entry.level = data["ready"].as<bool>() ? 0 : 1;
    entry.message = configuring_ ? "configuration mode: neutral output" :
      data["error"].as<std::string>().empty() ? "connected" : data["error"].as<std::string>();
    diagnostics.status.push_back(entry);
    diag_pub_->publish(diagnostics);
  }

  std::string profile_path_, input_dir_, error_;
  Profile profile_;
  bool profile_valid_{false}, configuring_{false}, syncing_{false};
  uint64_t generation_{0};
  std::unique_ptr<Device> device_;
  std::vector<float> raw_axes_ = std::vector<float>(kAxisCount, 0.0F);
  std::vector<int32_t> raw_keys_ = std::vector<int32_t>(kKeyCount, 0);
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_, raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_, ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr configure_;
  rclcpp::TimerBase::SharedPtr publish_timer_, scan_timer_, status_timer_;
};
}  // namespace kart_joy

RCLCPP_COMPONENTS_REGISTER_NODE(kart_joy::JoyNode)
