#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace kart_joy
{
constexpr int kAxisCount = 64;
constexpr int kKeyCount = 768;
inline const std::vector<std::string> axis_names{
  "left_x", "left_y", "right_x", "right_y", "l2", "r2", "dpad_x", "dpad_y"};
inline const std::vector<std::string> button_names{
  "cross", "circle", "triangle", "square", "l1", "r1", "l2", "r2",
  "create", "options", "ps", "l3", "r3", "dpad_up", "dpad_down", "dpad_left", "dpad_right"};

struct AxisBinding
{
  int code{-1};
  bool unipolar{false};
  double negative{-1.0}, center{0.0}, positive{1.0}, deadzone{0.05};
  void validate() const
  {
    if (code < -1 || code >= kAxisCount || !std::isfinite(negative) ||
      !std::isfinite(center) || !std::isfinite(positive) || !std::isfinite(deadzone) ||
      std::abs(negative) > 1.0 || std::abs(center) > 1.0 || std::abs(positive) > 1.0 ||
      deadzone < 0.0 || deadzone >= 1.0 || std::abs(positive - center) < 1e-6 ||
      (!unipolar && (negative - center) * (positive - center) >= -1e-6))
    {
      throw std::invalid_argument("invalid axis calibration");
    }
  }
  float map(double raw) const
  {
    if (code == -1 || !std::isfinite(raw)) {return 0.0F;}
    double value = (raw - center) / (positive - center);
    if (!unipolar && value < 0.0) {value = -(raw - center) / (negative - center);}
    value = std::clamp(value, unipolar ? 0.0 : -1.0, 1.0);
    if (std::abs(value) <= deadzone) {return 0.0F;}
    return static_cast<float>(std::copysign((std::abs(value) - deadzone) / (1.0 - deadzone), value));
  }
};

struct ButtonBinding
{
  int code{-1};
  bool from_axis{false};
  int direction{1};
  double threshold{0.5};
  void validate() const
  {
    if (code < -1 || code >= (from_axis ? kAxisCount : kKeyCount) ||
      (direction != 1 && direction != -1) || !std::isfinite(threshold) ||
      threshold <= 0.0 || threshold > 1.0)
    {
      throw std::invalid_argument("invalid button binding");
    }
  }
};

// A disconnect always produces neutral outputs, even when raw zero would map
// to a half-pressed trigger. Connected raw state is never synthesized as zero.
struct MappedState
{
  std::vector<float> axes = std::vector<float>(axis_names.size(), 0.0F);
  std::vector<int32_t> buttons = std::vector<int32_t>(button_names.size(), 0);
};
inline MappedState map_state(
  bool connected, const std::vector<float> & axes, const std::vector<int32_t> & keys,
  const std::vector<AxisBinding> & axis_map, const std::vector<ButtonBinding> & button_map)
{
  MappedState out;
  if (!connected) {return out;}
  for (size_t i = 0; i < axis_map.size() && i < out.axes.size(); ++i) {
    const auto & binding = axis_map[i];
    if (binding.code >= 0 && static_cast<size_t>(binding.code) < axes.size()) {
      out.axes[i] = binding.map(axes[binding.code]);
    }
  }
  for (size_t i = 0; i < button_map.size() && i < out.buttons.size(); ++i) {
    const auto & binding = button_map[i];
    if (binding.code < 0) {continue;}
    if (binding.from_axis && static_cast<size_t>(binding.code) < axes.size()) {
      out.buttons[i] = axes[binding.code] * binding.direction >= binding.threshold;
    } else if (!binding.from_axis && static_cast<size_t>(binding.code) < keys.size()) {
      out.buttons[i] = keys[binding.code] != 0;
    }
  }
  return out;
}
}  // namespace kart_joy
