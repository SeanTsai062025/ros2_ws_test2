#ifndef DEXTER_HARDWARE__MKS_PROTOCOL_HPP_
#define DEXTER_HARDWARE__MKS_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace dexter_hardware
{

constexpr std::uint8_t kReadEncoderCommand = 0x31;
constexpr std::uint8_t kAbsoluteAxisCommand = 0xF5;
constexpr std::uint8_t kSpeedModeCommand = 0xF6;
constexpr std::int32_t kMaxAxisTicks = 8388607;
constexpr std::uint16_t kProtocolMaxSpeed = 3000;

struct CanFrame
{
  std::uint32_t id{};
  std::vector<std::uint8_t> data;
};

struct MotorCalibration
{
  std::string joint_name;
  std::uint32_t can_id{};
  std::int64_t encoder_ticks_per_rev{16384};
  double gear_ratio{1.0};
  int command_direction{1};
  int encoder_direction{1};
  double speed_field_scale{1.0};
};

struct AxisConversion
{
  std::int32_t ticks{};
  bool clamped{false};
};

struct CommandSeed
{
  std::vector<double> positions;
  std::vector<double> velocities;
};

std::uint8_t checksum(std::uint32_t can_id, const std::vector<std::uint8_t> & payload);
bool has_valid_checksum(const CanFrame & frame);
std::int64_t decode_signed_big_endian(const std::uint8_t * bytes, std::size_t count);
std::array<std::uint8_t, 3> encode_signed_24(std::int32_t value);
std::optional<std::int64_t> parse_encoder_response(
  const CanFrame & frame, std::uint32_t expected_can_id);

CanFrame make_encoder_request(std::uint32_t can_id);
CanFrame make_absolute_axis_command(
  std::uint32_t can_id, std::uint16_t speed, std::uint8_t acceleration,
  std::int32_t target_ticks);
CanFrame make_speed_command(
  std::uint32_t can_id, double signed_speed_field, std::uint8_t acceleration);

double ticks_to_radians(std::int64_t ticks, const MotorCalibration & calibration);
AxisConversion radians_to_ticks(double radians, const MotorCalibration & calibration);
std::uint16_t velocity_to_speed_field(
  double joint_velocity_rad_s, const MotorCalibration & calibration,
  std::uint16_t min_speed, std::uint16_t fallback_speed, std::uint16_t max_speed);
bool absolute_command_changed(
  const std::optional<std::int32_t> & last_ticks,
  const std::optional<std::uint16_t> & last_speed,
  std::int32_t target_ticks, std::uint16_t speed);

std::vector<MotorCalibration> default_motor_calibrations();
CommandSeed make_activation_command_seed(const std::vector<double> & measured_positions);

}  // namespace dexter_hardware

#endif  // DEXTER_HARDWARE__MKS_PROTOCOL_HPP_
