#include "dexter_hardware/mks_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dexter_hardware
{
namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;
}

std::uint8_t checksum(const std::uint32_t can_id, const std::vector<std::uint8_t> & payload)
{
  std::uint32_t total = can_id;
  for (const auto byte : payload)
  {
    total += byte;
  }
  return static_cast<std::uint8_t>(total & 0xFFU);
}

bool has_valid_checksum(const CanFrame & frame)
{
  if (frame.data.size() < 2U)
  {
    return false;
  }
  const std::vector<std::uint8_t> payload(frame.data.begin(), frame.data.end() - 1);
  return checksum(frame.id, payload) == frame.data.back();
}

std::int64_t decode_signed_big_endian(const std::uint8_t * bytes, const std::size_t count)
{
  if (bytes == nullptr || count == 0U || count > sizeof(std::int64_t))
  {
    return 0;
  }

  std::uint64_t raw = 0;
  for (std::size_t index = 0; index < count; ++index)
  {
    raw = (raw << 8U) | bytes[index];
  }

  const std::size_t bits = count * 8U;
  if (bits < 64U && (raw & (std::uint64_t{1} << (bits - 1U))) != 0U)
  {
    raw |= (~std::uint64_t{0}) << bits;
  }
  return static_cast<std::int64_t>(raw);
}

std::array<std::uint8_t, 3> encode_signed_24(const std::int32_t value)
{
  const auto raw = static_cast<std::uint32_t>(value) & 0x00FFFFFFU;
  return {
    static_cast<std::uint8_t>((raw >> 16U) & 0xFFU),
    static_cast<std::uint8_t>((raw >> 8U) & 0xFFU),
    static_cast<std::uint8_t>(raw & 0xFFU)};
}

std::optional<std::int64_t> parse_encoder_response(
  const CanFrame & frame, const std::uint32_t expected_can_id)
{
  if (
    frame.id != expected_can_id || frame.data.size() != 8U ||
    frame.data.front() != kReadEncoderCommand || !has_valid_checksum(frame))
  {
    return std::nullopt;
  }
  return decode_signed_big_endian(frame.data.data() + 1, 6U);
}

CanFrame make_encoder_request(const std::uint32_t can_id)
{
  CanFrame frame{can_id, {kReadEncoderCommand}};
  frame.data.push_back(checksum(can_id, frame.data));
  return frame;
}

CanFrame make_absolute_axis_command(
  const std::uint32_t can_id, const std::uint16_t speed, const std::uint8_t acceleration,
  const std::int32_t target_ticks)
{
  const auto bounded_speed = std::min(speed, kProtocolMaxSpeed);
  const auto bounded_ticks = std::clamp(target_ticks, -kMaxAxisTicks, kMaxAxisTicks);
  const auto axis = encode_signed_24(bounded_ticks);
  CanFrame frame{
    can_id,
    {kAbsoluteAxisCommand,
      static_cast<std::uint8_t>((bounded_speed >> 8U) & 0x0FU),
      static_cast<std::uint8_t>(bounded_speed & 0xFFU), acceleration,
      axis[0], axis[1], axis[2]}};
  frame.data.push_back(checksum(can_id, frame.data));
  return frame;
}

CanFrame make_speed_command(
  const std::uint32_t can_id, const double signed_speed_field,
  const std::uint8_t acceleration)
{
  const auto magnitude = static_cast<std::uint16_t>(std::clamp(
    std::llround(std::abs(signed_speed_field)), 0LL,
    static_cast<long long>(kProtocolMaxSpeed)));
  const std::uint8_t direction = signed_speed_field < 0.0 ? 0x80U : 0x00U;
  CanFrame frame{
    can_id,
    {kSpeedModeCommand,
      static_cast<std::uint8_t>(direction | ((magnitude >> 8U) & 0x0FU)),
      static_cast<std::uint8_t>(magnitude & 0xFFU), acceleration}};
  frame.data.push_back(checksum(can_id, frame.data));
  return frame;
}

double ticks_to_radians(const std::int64_t ticks, const MotorCalibration & calibration)
{
  return static_cast<double>(ticks) * kTwoPi * calibration.encoder_direction /
         (static_cast<double>(calibration.encoder_ticks_per_rev) * calibration.gear_ratio);
}

AxisConversion radians_to_ticks(const double radians, const MotorCalibration & calibration)
{
  if (!std::isfinite(radians))
  {
    return {0, true};
  }
  const auto unbounded = std::llround(
    radians / kTwoPi * static_cast<double>(calibration.encoder_ticks_per_rev) *
    calibration.gear_ratio * calibration.command_direction);
  const auto bounded = std::clamp(
    unbounded, -static_cast<long long>(kMaxAxisTicks),
    static_cast<long long>(kMaxAxisTicks));
  return {static_cast<std::int32_t>(bounded), bounded != unbounded};
}

std::uint16_t velocity_to_speed_field(
  const double joint_velocity_rad_s, const MotorCalibration & calibration,
  const std::uint16_t min_speed, const std::uint16_t fallback_speed,
  const std::uint16_t max_speed)
{
  if (!std::isfinite(joint_velocity_rad_s) || std::abs(joint_velocity_rad_s) < 1.0e-6)
  {
    return std::min(fallback_speed, max_speed);
  }

  const double motor_rpm =
    std::abs(joint_velocity_rad_s) * calibration.gear_ratio * 60.0 / kTwoPi;
  const auto scaled = static_cast<long long>(
    std::llround(motor_rpm * calibration.speed_field_scale));
  return static_cast<std::uint16_t>(std::clamp(
    scaled, static_cast<long long>(min_speed), static_cast<long long>(max_speed)));
}

bool absolute_command_changed(
  const std::optional<std::int32_t> & last_ticks,
  const std::optional<std::uint16_t> & last_speed,
  const std::int32_t target_ticks, const std::uint16_t speed)
{
  return !last_ticks || !last_speed || *last_ticks != target_ticks || *last_speed != speed;
}

std::vector<MotorCalibration> default_motor_calibrations()
{
  return {
    {"base", 1U, 16384, 30.0, 1, 1, 1.0},
    {"part1", 2U, 16384, 30.0, -1, 1, 1.0},
    {"part2", 3U, 16384, 30.0, -1, -1, 1.0},
    {"part3", 4U, 16384, 30.0, 1, 1, 1.0},
    {"part4", 5U, 16384, 30.0, 1, 1, 1.0},
    {"part5", 6U, 16384, 1.0, -1, -1, 8.0}};
}

CommandSeed make_activation_command_seed(const std::vector<double> & measured_positions)
{
  return {measured_positions, std::vector<double>(measured_positions.size(), 0.0)};
}

}  // namespace dexter_hardware
