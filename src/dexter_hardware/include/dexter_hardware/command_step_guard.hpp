#ifndef DEXTER_HARDWARE__COMMAND_STEP_GUARD_HPP_
#define DEXTER_HARDWARE__COMMAND_STEP_GUARD_HPP_

#include <cmath>

namespace dexter_hardware
{

enum class CommandStepDecision
{
  ACCEPT,
  REBASE_TO_MEASUREMENT,
  REJECT
};

inline CommandStepDecision evaluate_command_step(
  const double last_command, const double command, const double measured_position,
  const double max_step) noexcept
{
  if (
    !std::isfinite(last_command) || !std::isfinite(command) ||
    !std::isfinite(measured_position) || !std::isfinite(max_step) || max_step <= 0.0)
  {
    return CommandStepDecision::REJECT;
  }

  if (std::abs(command - last_command) <= max_step)
  {
    return CommandStepDecision::ACCEPT;
  }

  // MoveIt starts each newly planned trajectory at the latest physical state.
  // If an idle/backdrivable joint moved since the previous trajectory, JTC's
  // first reference legitimately changes from the stale last command to that
  // measured state. Sending a target that is already at the joint cannot cause
  // the discontinuous motion that the command-to-command limit prevents.
  if (std::abs(command - measured_position) <= max_step)
  {
    return CommandStepDecision::REBASE_TO_MEASUREMENT;
  }

  return CommandStepDecision::REJECT;
}

}  // namespace dexter_hardware

#endif  // DEXTER_HARDWARE__COMMAND_STEP_GUARD_HPP_
