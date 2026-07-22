#include <gtest/gtest.h>

#include "dexter_hardware/control_cycle_guard.hpp"

namespace dexter_hardware
{
namespace
{

TEST(ControlCycleGuard, DefersActivationWriteUntilPostSwitchRead)
{
  ControlCycleGuard guard;

  guard.record_read();
  guard.require_post_switch_read();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::WAITING_FOR_READ);

  guard.record_read();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::READY);
  guard.record_write();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::STALE);
}

TEST(ControlCycleGuard, RepeatedDeferredWritesDoNotConsumeTheRequiredRead)
{
  ControlCycleGuard guard;

  guard.record_read();
  guard.require_post_switch_read();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::WAITING_FOR_READ);
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::WAITING_FOR_READ);
  EXPECT_TRUE(guard.awaiting_post_switch_read());

  guard.record_read();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::READY);
  EXPECT_FALSE(guard.awaiting_post_switch_read());
}

TEST(ControlCycleGuard, ResetRequiresAReadBeforeNormalWrite)
{
  ControlCycleGuard guard;

  guard.record_read();
  guard.record_write();
  guard.reset();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::STALE);

  guard.record_read();
  EXPECT_EQ(guard.evaluate_write(), WriteCycleDecision::READY);
}

}  // namespace
}  // namespace dexter_hardware
