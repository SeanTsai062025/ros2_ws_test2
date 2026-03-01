/*
 * test_angle_conversion.cpp  –  unit tests for the angle→ticks math
 *
 * These are off-target tests (no /dev/mem needed).
 */

#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

// Replicate the pure-math portion of the node so we can test it without
// bringing up ROS or needing /dev/mem.

struct ServoParams {
    double   min_pulse_us;
    double   max_pulse_us;
    double   frequency_hz;
    uint32_t clk_rate_hz;
    uint32_t range;  // clk_rate / frequency

    uint32_t angle_to_ticks(double angle_deg) const
    {
        angle_deg = std::clamp(angle_deg, 0.0, 180.0);
        double pulse_us = min_pulse_us +
            (angle_deg / 180.0) * (max_pulse_us - min_pulse_us);
        double ticks = pulse_us * static_cast<double>(clk_rate_hz) / 1.0e6;
        return static_cast<uint32_t>(std::round(ticks));
    }
};

class AngleConversionTest : public ::testing::Test {
protected:
    // Realistic values matching the RP1 clk_pwm0 @ ~6.144 MHz
    ServoParams p{500.0, 2500.0, 50.0, 6144003, 0};

    void SetUp() override {
        p.range = static_cast<uint32_t>(
            std::round(static_cast<double>(p.clk_rate_hz) / p.frequency_hz));
    }
};

TEST_F(AngleConversionTest, ZeroDegrees)
{
    // 0° → 500 µs pulse → 500e-6 * 6144003 ≈ 3072 ticks
    uint32_t ticks = p.angle_to_ticks(0.0);
    EXPECT_NEAR(ticks, 3072, 2);
}

TEST_F(AngleConversionTest, NinetyDegrees)
{
    // 90° → 1500 µs → 1500e-6 * 6144003 ≈ 9216 ticks
    uint32_t ticks = p.angle_to_ticks(90.0);
    EXPECT_NEAR(ticks, 9216, 2);
}

TEST_F(AngleConversionTest, OneEightyDegrees)
{
    // 180° → 2500 µs → 2500e-6 * 6144003 ≈ 15360 ticks
    uint32_t ticks = p.angle_to_ticks(180.0);
    EXPECT_NEAR(ticks, 15360, 2);
}

TEST_F(AngleConversionTest, Clamping)
{
    // Negative should clamp to 0°
    EXPECT_EQ(p.angle_to_ticks(-10.0), p.angle_to_ticks(0.0));
    // >180 should clamp to 180°
    EXPECT_EQ(p.angle_to_ticks(999.0), p.angle_to_ticks(180.0));
}

TEST_F(AngleConversionTest, RangeValue)
{
    // At 50 Hz:  6144003 / 50 = 122880.06 → 122880
    EXPECT_EQ(p.range, 122880u);
}

TEST_F(AngleConversionTest, DutyCycleBounds)
{
    // Duty for 0° must be < range  (500us / 20000us = 2.5%)
    EXPECT_LT(p.angle_to_ticks(0.0), p.range);
    // Duty for 180° must be < range (2500us / 20000us = 12.5%)
    EXPECT_LT(p.angle_to_ticks(180.0), p.range);
}
