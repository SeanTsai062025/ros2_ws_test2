#include "dexter_hardware/mks_can_client.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace dexter_hardware
{

MksCanClient::MksCanClient(std::unique_ptr<CanTransport> transport)
: transport_(std::move(transport))
{
}

MksCanClient::~MksCanClient()
{
  close();
}

bool MksCanClient::resynchronize(
  const std::chrono::microseconds quiet_period, const std::chrono::microseconds max_wait,
  std::string & error)
{
  synchronized_ = false;
  const auto overall_deadline = std::chrono::steady_clock::now() + max_wait;
  auto quiet_deadline = std::chrono::steady_clock::now() + quiet_period;

  while (std::chrono::steady_clock::now() < overall_deadline)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= quiet_deadline)
    {
      synchronized_ = true;
      error.clear();
      return true;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(quiet_deadline - now);
    std::string receive_error;
    auto frame = transport_->receive(remaining, receive_error);
    if (!receive_error.empty())
    {
      error = receive_error;
      return false;
    }
    if (frame)
    {
      ++counters_.drained;
      classify_ignored(*frame);
      quiet_deadline = std::chrono::steady_clock::now() + quiet_period;
    }
  }

  error = "CAN bus did not become quiet during synchronization window";
  return false;
}

bool MksCanClient::drain_pending(std::string & error)
{
  while (true)
  {
    std::string receive_error;
    auto frame = transport_->receive(std::chrono::microseconds{0}, receive_error);
    if (!receive_error.empty())
    {
      error = receive_error;
      return false;
    }
    if (!frame)
    {
      error.clear();
      return true;
    }
    ++counters_.drained;
    classify_ignored(*frame);
  }
}

void MksCanClient::classify_ignored(const CanFrame & frame)
{
  if (!has_valid_checksum(frame))
  {
    ++counters_.bad_checksum;
  }
  else if (!frame.data.empty() && frame.data.front() == kAbsoluteAxisCommand)
  {
    ++counters_.f5_status;
  }
  else
  {
    ++counters_.unrelated;
  }
}

std::optional<CanFrame> MksCanClient::wait_for_response(
  const std::uint32_t motor_id, const std::uint8_t command,
  const std::chrono::microseconds timeout, std::string & error)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - std::chrono::steady_clock::now());
    std::string receive_error;
    auto frame = transport_->receive(remaining, receive_error);
    if (!receive_error.empty())
    {
      error = receive_error;
      return std::nullopt;
    }
    if (!frame)
    {
      break;
    }
    if (
      frame->id == motor_id && !frame->data.empty() && frame->data.front() == command &&
      has_valid_checksum(*frame))
    {
      error.clear();
      return frame;
    }
    classify_ignored(*frame);
  }
  error = "timed out waiting for motor " + std::to_string(motor_id) +
          " response 0x" + [] (const std::uint8_t value) {
            constexpr char digits[] = "0123456789ABCDEF";
            std::string result;
            result.push_back(digits[(value >> 4U) & 0x0FU]);
            result.push_back(digits[value & 0x0FU]);
            return result;
          }(command);
  return std::nullopt;
}

std::optional<std::int64_t> MksCanClient::read_encoder(
  const std::uint32_t motor_id, const std::chrono::microseconds timeout,
  std::string & error)
{
  if (!synchronized_)
  {
    error = "CAN encoder transactions are not synchronized";
    return std::nullopt;
  }

  // 0x31 has no request sequence number. Discard everything already queued before
  // transmitting, and permit only one outstanding request. If it times out, latch
  // unsynchronized and require a lifecycle reconfigure/quiet interval before retrying.
  if (!drain_pending(error))
  {
    synchronized_ = false;
    return std::nullopt;
  }
  if (!transport_->send(make_encoder_request(motor_id), error))
  {
    synchronized_ = false;
    return std::nullopt;
  }

  auto frame = wait_for_response(motor_id, kReadEncoderCommand, timeout, error);
  if (!frame)
  {
    synchronized_ = false;
    return std::nullopt;
  }
  auto ticks = parse_encoder_response(*frame, motor_id);
  if (!ticks)
  {
    synchronized_ = false;
    error = "malformed encoder response from motor " + std::to_string(motor_id);
  }
  return ticks;
}

std::optional<std::uint32_t> MksCanClient::read_parameter(
  const std::uint32_t motor_id, const std::uint8_t parameter,
  const std::chrono::milliseconds timeout, std::string & error)
{
  if (!drain_pending(error))
  {
    return std::nullopt;
  }
  CanFrame request{motor_id, {0x00U, parameter}};
  request.data.push_back(checksum(motor_id, request.data));
  if (!transport_->send(request, error))
  {
    return std::nullopt;
  }
  auto response = wait_for_response(
    motor_id, parameter, std::chrono::duration_cast<std::chrono::microseconds>(timeout), error);
  if (!response || response->data.size() < 3U)
  {
    return std::nullopt;
  }
  if (
    response->data.size() == 4U && response->data[1] == 0xFFU &&
    response->data[2] == 0xFFU)
  {
    error = "motor returned unsupported parameter sentinel";
    return std::nullopt;
  }

  std::uint32_t value = 0U;
  for (std::size_t index = 1U; index + 1U < response->data.size(); ++index)
  {
    value = (value << 8U) | response->data[index];
  }
  return value;
}

bool MksCanClient::set_parameter(
  const std::uint32_t motor_id, const std::uint8_t parameter,
  const std::vector<std::uint8_t> & payload, const std::chrono::milliseconds timeout,
  std::string & error)
{
  if (!drain_pending(error))
  {
    return false;
  }
  CanFrame request{motor_id, {parameter}};
  request.data.insert(request.data.end(), payload.begin(), payload.end());
  request.data.push_back(checksum(motor_id, request.data));
  if (!transport_->send(request, error))
  {
    return false;
  }
  auto response = wait_for_response(
    motor_id, parameter, std::chrono::duration_cast<std::chrono::microseconds>(timeout), error);
  return response && response->data.size() >= 3U && response->data[1] == 1U;
}

bool MksCanClient::send_absolute(
  const std::uint32_t motor_id, const std::uint16_t speed,
  const std::uint8_t acceleration, const std::int32_t target_ticks,
  std::string & error)
{
  return transport_->send(
    make_absolute_axis_command(motor_id, speed, acceleration, target_ticks), error);
}

bool MksCanClient::send_stop(const std::uint32_t motor_id, std::string & error)
{
  return transport_->send(make_speed_command(motor_id, 0.0, 0U), error);
}

void MksCanClient::close() noexcept
{
  if (transport_)
  {
    transport_->close();
  }
  synchronized_ = false;
}

}  // namespace dexter_hardware
