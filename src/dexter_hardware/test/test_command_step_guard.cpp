#include <limits>

#include <gtest/gtest.h>

#include "dexter_hardware/command_step_guard.hpp"

namespace dexter_hardware
{
namespace
{

TEST(CommandStepGuard, AcceptsNormalSampledTrajectoryStep)
{
  EXPECT_EQ(
    evaluate_command_step(0.10, 0.12, 0.09, 0.05),
    CommandStepDecision::ACCEPT);
}

TEST(CommandStepGuard, RebasesNewTrajectoryAtFreshMeasuredPosition)
{
  // Regression for the observed Part 5 failure. The previous trajectory
  // finished near zero, but the next trajectory legitimately began at the
  // latest encoder position after the idle joint had moved.
  EXPECT_EQ(
    evaluate_command_step(0.000099, -0.465863, -0.465860, 0.05),
    CommandStepDecision::REBASE_TO_MEASUREMENT);
}

TEST(CommandStepGuard, RejectsLargeTargetAwayFromBothCommandAndEncoder)
{
  EXPECT_EQ(
    evaluate_command_step(0.000099, -0.465863, 0.001, 0.05),
    CommandStepDecision::REJECT);
}

TEST(CommandStepGuard, RejectsInvalidInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    evaluate_command_step(nan, 0.0, 0.0, 0.05),
    CommandStepDecision::REJECT);
  EXPECT_EQ(
    evaluate_command_step(0.0, 0.0, 0.0, 0.0),
    CommandStepDecision::REJECT);
}

}  // namespace
}  // namespace dexter_hardware
