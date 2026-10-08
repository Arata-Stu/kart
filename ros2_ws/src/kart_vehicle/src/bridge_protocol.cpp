#include "kart_vehicle/bridge_protocol.hpp"

#include <algorithm>
#include <charconv>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace kart_vehicle
{
namespace
{

void set_error(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

std::vector<std::string_view> split(const std::string_view input, const char delimiter)
{
  std::vector<std::string_view> fields;
  std::size_t begin = 0;
  while (begin <= input.size()) {
    const auto end = input.find(delimiter, begin);
    if (end == std::string_view::npos) {
      fields.emplace_back(input.substr(begin));
      break;
    }
    fields.emplace_back(input.substr(begin, end - begin));
    begin = end + 1;
  }
  return fields;
}

template<typename IntegerT>
bool parse_integer(const std::string_view text, IntegerT & output, const int base = 10)
{
  if (text.empty()) {
    return false;
  }
  const auto result = std::from_chars(text.data(), text.data() + text.size(), output, base);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

bool pulse_is_valid(const int pulse_us)
{
  return pulse_us == 0 || (pulse_us >= 800 && pulse_us <= 2200);
}

float clamp_normalized(const double value)
{
  return static_cast<float>(std::clamp(value, 0.0, 1.0));
}

}  // namespace

std::uint16_t crc16_ccitt(const std::string & data)
{
  std::uint16_t crc = 0xFFFFU;
  for (const auto byte : data) {
    crc ^= static_cast<std::uint16_t>(static_cast<std::uint8_t>(byte)) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::string encode_command(const CommandFrame & frame)
{
  constexpr std::uint16_t valid_flags = COMMAND_VALID | HOST_REQUEST;
  if (
    frame.steering_milli < -1000 || frame.steering_milli > 1000 ||
    frame.throttle_milli < 0 || frame.throttle_milli > 1000 ||
    frame.reverse_milli < 0 || frame.reverse_milli > 1000 ||
    frame.brake_milli < 0 || frame.brake_milli > 1000 ||
    (frame.throttle_milli > 0 && frame.reverse_milli > 0) ||
    (frame.brake_milli > 0 && (frame.throttle_milli > 0 || frame.reverse_milli > 0)) ||
    (frame.flags & static_cast<std::uint16_t>(~valid_flags)) != 0U)
  {
    throw std::invalid_argument("command frame value is out of range");
  }

  std::ostringstream payload;
  payload << kProtocolVersion << ",C,"
          << frame.sequence << ','
          << frame.steering_milli << ','
          << frame.throttle_milli << ','
          << frame.reverse_milli << ','
          << frame.brake_milli << ','
          << frame.flags;

  const auto payload_text = payload.str();
  std::ostringstream frame_stream;
  frame_stream << payload_text << ','
               << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
               << crc16_ccitt(payload_text) << '\n';
  return frame_stream.str();
}

std::optional<StatusFrame> parse_status(const std::string & line, std::string * error)
{
  auto frame_text = line;
  while (!frame_text.empty() && (frame_text.back() == '\n' || frame_text.back() == '\r')) {
    frame_text.pop_back();
  }

  const auto crc_separator = frame_text.rfind(',');
  if (crc_separator == std::string::npos) {
    set_error(error, "missing CRC field");
    return std::nullopt;
  }

  const auto payload = frame_text.substr(0, crc_separator);
  const auto crc_text = std::string_view(frame_text).substr(crc_separator + 1);
  std::uint16_t received_crc = 0;
  if (crc_text.size() != 4 || !parse_integer(crc_text, received_crc, 16)) {
    set_error(error, "invalid CRC field");
    return std::nullopt;
  }
  if (received_crc != crc16_ccitt(payload)) {
    set_error(error, "CRC mismatch");
    return std::nullopt;
  }

  const auto fields = split(payload, ',');
  if (fields.size() != 12 || fields[0] != kProtocolVersion || fields[1] != "S") {
    set_error(error, "unexpected status frame format");
    return std::nullopt;
  }

  StatusFrame frame;
  unsigned int selector = 0;
  unsigned int active_path = 0;
  if (
    !parse_integer(fields[2], frame.sequence) ||
    !parse_integer(fields[3], frame.rx1_us) ||
    !parse_integer(fields[4], frame.rx2_us) ||
    !parse_integer(fields[5], frame.rx3_us) ||
    !parse_integer(fields[6], frame.servo_output_us) ||
    !parse_integer(fields[7], frame.esc_output_us) ||
    !parse_integer(fields[8], frame.vbec_mv) ||
    !parse_integer(fields[9], selector) ||
    !parse_integer(fields[10], active_path) ||
    !parse_integer(fields[11], frame.fault_bits))
  {
    set_error(error, "status frame contains a non-integer field");
    return std::nullopt;
  }

  if (
    !pulse_is_valid(frame.rx1_us) ||
    !pulse_is_valid(frame.rx2_us) ||
    !pulse_is_valid(frame.rx3_us) ||
    !pulse_is_valid(frame.servo_output_us) ||
    !pulse_is_valid(frame.esc_output_us) ||
    frame.vbec_mv < 0 || frame.vbec_mv > 20000 ||
    selector > static_cast<unsigned int>(RcSelector::automatic) ||
    active_path > static_cast<unsigned int>(ActivePath::failsafe))
  {
    set_error(error, "status frame value is out of range");
    return std::nullopt;
  }

  frame.selector = static_cast<RcSelector>(selector);
  frame.active_path = static_cast<ActivePath>(active_path);
  return frame;
}

bool propo_calibration_is_valid(const PropoCalibration & calibration)
{
  constexpr int minimum_pulse_us = 800;
  constexpr int maximum_pulse_us = 2200;
  return calibration.steering_left_us >= minimum_pulse_us &&
         calibration.steering_right_us <= maximum_pulse_us &&
         calibration.throttle_forward_us >= minimum_pulse_us &&
         calibration.throttle_reverse_us <= maximum_pulse_us &&
         calibration.steering_left_us < calibration.steering_neutral_us &&
         calibration.steering_neutral_us < calibration.steering_right_us &&
         calibration.throttle_forward_us < calibration.throttle_neutral_us &&
         calibration.throttle_neutral_us < calibration.throttle_reverse_us;
}

std::optional<PropoCommand> propo_pwm_to_command(
  const int steering_us, const int throttle_us, const PropoCalibration & calibration)
{
  if (!propo_calibration_is_valid(calibration) || !pulse_is_valid(steering_us) ||
    !pulse_is_valid(throttle_us) || steering_us == 0 || throttle_us == 0)
  {
    return std::nullopt;
  }

  PropoCommand command;
  if (steering_us <= calibration.steering_neutral_us)
  {
    command.steering = clamp_normalized(
      static_cast<double>(calibration.steering_neutral_us - steering_us) /
      static_cast<double>(calibration.steering_neutral_us - calibration.steering_left_us));
  }
  else
  {
    command.steering = -clamp_normalized(
      static_cast<double>(steering_us - calibration.steering_neutral_us) /
      static_cast<double>(calibration.steering_right_us - calibration.steering_neutral_us));
  }

  if (throttle_us <= calibration.throttle_neutral_us)
  {
    command.throttle = clamp_normalized(
      static_cast<double>(calibration.throttle_neutral_us - throttle_us) /
      static_cast<double>(calibration.throttle_neutral_us - calibration.throttle_forward_us));
  }
  else
  {
    command.reverse = clamp_normalized(
      static_cast<double>(throttle_us - calibration.throttle_neutral_us) /
      static_cast<double>(calibration.throttle_reverse_us - calibration.throttle_neutral_us));
  }
  return command;
}

}  // namespace kart_vehicle
