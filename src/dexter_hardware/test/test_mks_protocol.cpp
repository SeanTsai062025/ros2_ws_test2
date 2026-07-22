#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "dexter_hardware/mks_protocol.hpp"

namespace dexter_hardware
{
namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;

CanFrame encoder_response(const std::uint32_t id, const std::array<std::uint8_t, 6> & raw)
{
  CanFrame frame{id, {kReadEncoderCommand}};
  frame.data.insert(frame.data.end(), raw.begin(), raw.end());
  frame.data.push_back(checksum(id, frame.data));
  return frame;
}

TEST(MksProtocol, ChecksumCoversCanIdAndPayload)
{
  EXPECT_EQ(checksum(1U, {0x31U}), 0x32U);
  auto request = make_encoder_request(6U);
  ASSERT_EQ(request.data.size(), 2U);
  EXPECT_TRUE(has_valid_checksum(request));
  request.data.back() ^= 0x01U;
  EXPECT_FALSE(has_valid_checksum(request));
}

TEST(MksProtocol, Signed24EncodingPreservesTwosComplement)
{
  EXPECT_EQ(encode_signed_24(8388607), (std::array<std::uint8_t, 3>{0x7F, 0xFF, 0xFF}));
  EXPECT_EQ(encode_signed_24(-8388608), (std::array<std::uint8_t, 3>{0x80, 0x00, 0x00}));
  EXPECT_EQ(encode_signed_24(-1), (std::array<std::uint8_t, 3>{0xFF, 0xFF, 0xFF}));
}

TEST(MksProtocol, Signed48EncoderValuesDecodeAtLimits)
{
  auto positive = encoder_response(3U, {0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
  auto negative = encoder_response(3U, {0x80, 0x00, 0x00, 0x00, 0x00, 0x00});
  EXPECT_EQ(parse_encoder_response(positive, 3U), (std::int64_t{1} << 47U) - 1);
  EXPECT_EQ(parse_encoder_response(negative, 3U), -(std::int64_t{1} << 47U));
  positive.data.back() ^= 0xFFU;
  EXPECT_FALSE(parse_encoder_response(positive, 3U));
  EXPECT_FALSE(parse_encoder_response(negative, 2U));
}

TEST(MksProtocol, CalibratedGearRatiosAndDirectionsArePreserved)
{
  const auto motors = default_motor_calibrations();
  ASSERT_EQ(motors.size(), 6U);
  EXPECT_EQ(radians_to_ticks(kTwoPi, motors[0]).ticks, 16384 * 30);
  EXPECT_EQ(radians_to_ticks(kTwoPi, motors[1]).ticks, -16384 * 30);
  EXPECT_EQ(radians_to_ticks(kTwoPi, motors[2]).ticks, -16384 * 30);
  EXPECT_EQ(radians_to_ticks(kTwoPi, motors[3]).ticks, 16384 * 30);
  EXPECT_EQ(radians_to_ticks(kTwoPi, motors[4]).ticks, 16384 * 30);
  EXPECT_EQ(radians_to_ticks(kTwoPi, motors[5]).ticks, -16384);

  EXPECT_NEAR(ticks_to_radians(16384 * 30, motors[0]), kTwoPi, 1.0e-12);
  EXPECT_NEAR(ticks_to_radians(16384 * 30, motors[1]), kTwoPi, 1.0e-12);
  EXPECT_NEAR(ticks_to_radians(16384 * 30, motors[2]), -kTwoPi, 1.0e-12);
  EXPECT_NEAR(ticks_to_radians(16384, motors[5]), -kTwoPi, 1.0e-12);
}

TEST(MksProtocol, AxisAndSpeedCommandLimitsAreEnforced)
{
  const auto motors = default_motor_calibrations();
  const auto too_large = radians_to_ticks(1.0e9, motors[0]);
  EXPECT_TRUE(too_large.clamped);
  EXPECT_EQ(too_large.ticks, kMaxAxisTicks);

  const auto frame = make_absolute_axis_command(1U, 65000U, 255U, std::numeric_limits<int>::min());
  ASSERT_EQ(frame.data.size(), 8U);
  EXPECT_EQ(frame.data[1], 0x0BU);
  EXPECT_EQ(frame.data[2], 0xB8U);
  EXPECT_EQ(frame.data[4], 0x80U);
  EXPECT_EQ(frame.data[5], 0x00U);
  EXPECT_EQ(frame.data[6], 0x01U);
  EXPECT_TRUE(has_valid_checksum(frame));

  EXPECT_EQ(velocity_to_speed_field(0.0, motors[0], 10U, 300U, 3000U), 300U);
  EXPECT_EQ(velocity_to_speed_field(1.0, motors[5], 1U, 1U, 3000U), 76U);
  EXPECT_EQ(velocity_to_speed_field(1.0e6, motors[5], 1U, 1U, 3000U), 3000U);
}

TEST(MksProtocol, ActivationCommandsStartAtMeasuredPositionsNotZero)
{
  const std::vector<double> measured{0.42, -0.17, 1.1, -1.2, 0.03, 2.4};
  const auto seed = make_activation_command_seed(measured);
  EXPECT_EQ(seed.positions, measured);
  EXPECT_EQ(seed.velocities, std::vector<double>(measured.size(), 0.0));
  EXPECT_NE(seed.positions, std::vector<double>(measured.size(), 0.0));
}

TEST(MksProtocol, UnchangedIdleTargetDoesNotGenerateRepeatedF5Command)
{
  EXPECT_FALSE(absolute_command_changed(123, 1U, 123, 1U));
  EXPECT_TRUE(absolute_command_changed(123, 1U, 124, 1U));
  EXPECT_TRUE(absolute_command_changed(123, 1U, 123, 2U));
  EXPECT_TRUE(absolute_command_changed(std::nullopt, std::nullopt, 123, 1U));
}

}  // namespace
}  // namespace dexter_hardware
