#ifndef DEXTER_HARDWARE__CONTROL_CYCLE_GUARD_HPP_
#define DEXTER_HARDWARE__CONTROL_CYCLE_GUARD_HPP_

#include <cstdint>

namespace dexter_hardware
{

enum class WriteCycleDecision
{
  READY,
  WAITING_FOR_READ,
  STALE
};

// Enforces read -> controller update -> write while allowing ros2_control's
// expected write call in the same cycle as a command-interface mode switch.
class ControlCycleGuard
{
public:
  void reset() noexcept
  {
    read_generation_ = 0U;
    write_generation_ = 0U;
    post_switch_generation_ = 0U;
    awaiting_post_switch_read_ = false;
  }

  void record_read() noexcept
  {
    ++read_generation_;
  }

  void require_post_switch_read() noexcept
  {
    post_switch_generation_ = read_generation_;
    awaiting_post_switch_read_ = true;
  }

  WriteCycleDecision evaluate_write() noexcept
  {
    if (awaiting_post_switch_read_)
    {
      if (read_generation_ <= post_switch_generation_)
      {
        return WriteCycleDecision::WAITING_FOR_READ;
      }
      awaiting_post_switch_read_ = false;
    }
    return read_generation_ > write_generation_ ?
      WriteCycleDecision::READY : WriteCycleDecision::STALE;
  }

  void record_write() noexcept
  {
    write_generation_ = read_generation_;
  }

  std::uint64_t read_generation() const noexcept { return read_generation_; }
  std::uint64_t write_generation() const noexcept { return write_generation_; }
  bool awaiting_post_switch_read() const noexcept { return awaiting_post_switch_read_; }

private:
  std::uint64_t read_generation_{0U};
  std::uint64_t write_generation_{0U};
  std::uint64_t post_switch_generation_{0U};
  bool awaiting_post_switch_read_{false};
};

}  // namespace dexter_hardware

#endif  // DEXTER_HARDWARE__CONTROL_CYCLE_GUARD_HPP_
