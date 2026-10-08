#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace kart_vehicle
{

constexpr char kProtocolVersion[] = "JPB1";

enum CommandFlag : std::uint16_t
{
  COMMAND_VALID = 1U << 0,
  HOST_REQUEST = 1U << 1,
  // Backward-compatible spelling. This bit selects the STM32 host path for
  // both joystick MANUAL and autonomous AUTO control.
  AUTO_REQUEST = HOST_REQUEST,
};

enum class RcSelector : std::uint8_t
{
  propo = 0,
  automatic = 1,
};

enum class ActivePath : std::uint8_t
{
  disabled = 0,
  manual = 1,
  automatic = 2,
  failsafe = 3,
};

struct CommandFrame
{
  std::uint32_t sequence{0};
  int steering_milli{0};
  int throttle_milli{0};
  int reverse_milli{0};
  int brake_milli{0};
  std::uint16_t flags{0};
};

struct StatusFrame
{
  std::uint32_t sequence{0};
  int rx1_us{0};
  int rx2_us{0};
  int rx3_us{0};
  int servo_output_us{0};
  int esc_output_us{0};
  int vbec_mv{0};
  RcSelector selector{RcSelector::propo};
  ActivePath active_path{ActivePath::disabled};
  std::uint32_t fault_bits{0};
};

struct PropoCalibration
{
  int steering_left_us{1000};
  int steering_neutral_us{1500};
  int steering_right_us{2000};
  int throttle_forward_us{1000};
  int throttle_neutral_us{1500};
  int throttle_reverse_us{2000};
};

struct PropoCommand
{
  float steering{0.0F};
  float throttle{0.0F};
  float reverse{0.0F};
  float brake{0.0F};
};

std::uint16_t crc16_ccitt(const std::string & data);
std::string encode_command(const CommandFrame & frame);
std::optional<StatusFrame> parse_status(const std::string & line, std::string * error = nullptr);
bool propo_calibration_is_valid(const PropoCalibration & calibration);
std::optional<PropoCommand> propo_pwm_to_command(
  int steering_us, int throttle_us, const PropoCalibration & calibration);

}  // namespace kart_vehicle
