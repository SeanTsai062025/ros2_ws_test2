#include "dexter_hardware/mks_can_client.hpp"

#include <algorithm>
#include <chrono>
#include <iterator>
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
  const auto samples = read_encoders({motor_id}, timeout, error);
  if (!samples)
  {
    return std::nullopt;
  }
  return samples->front().ticks;
}

std::optional<std::vector<EncoderSample>> MksCanClient::read_encoders(
  const std::vector<std::uint32_t> & motor_ids,
  const std::chrono::microseconds batch_timeout, std::string & error,
  const std::size_t max_outstanding_requests,
  const std::chrono::microseconds request_spacing)
{
  if (!synchronized_)
  {
    error = "CAN encoder transactions are not synchronized";
    return std::nullopt;
  }
  if (
    motor_ids.empty() || batch_timeout.count() <= 0 || max_outstanding_requests == 0U ||
    request_spacing.count() < 0)
  {
    error = "encoder batch requires motors, a positive timeout, and a valid request window";
    return std::nullopt;
  }
  for (auto iterator = motor_ids.begin(); iterator != motor_ids.end(); ++iterator)
  {
    if (std::find(motor_ids.begin(), iterator, *iterator) != iterator)
    {
      error = "encoder batch contains duplicate motor ID " + std::to_string(*iterator);
      return std::nullopt;
    }
  }

  // 0x31 has no request sequence number. Discard everything queued before this batch,
  // allow only one request per motor, and never start another batch after a timeout.
  // The MKS nodes answer immediately with their command CAN ID. Requesting all six at
  // once makes multiple controllers begin transmitting together and caused repeatable
  // CAN protocol-error bursts on the physical daisy chain. A hardware capture showed
  // that even three-request windows can drive the adapter error-passive and lose the
  // last response. The production configuration therefore keeps exactly one MKS
  // request outstanding; larger windows and pacing remain available for diagnostics.
  const auto deadline = std::chrono::steady_clock::now() + batch_timeout;
  if (!drain_pending(error))
  {
    synchronized_ = false;
    return std::nullopt;
  }

  std::vector<std::optional<EncoderSample>> pending_samples(motor_ids.size());
  // A userspace scheduling pause can end after the deadline even though every CAN
  // reply reached the kernel socket queue on time. Once the blocking deadline
  // expires, perform a bounded non-blocking drain of the active window before
  // declaring a motor missing. The bound prevents unrelated traffic from extending
  // a control cycle indefinitely.
  constexpr std::size_t kMaxPostDeadlineFrames = 64U;
  std::size_t post_deadline_frames = 0U;

  for (std::size_t window_begin = 0U; window_begin < motor_ids.size();
    window_begin += max_outstanding_requests)
  {
    const auto window_end = std::min(
      motor_ids.size(), window_begin + max_outstanding_requests);
    for (std::size_t index = window_begin; index < window_end; ++index)
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        synchronized_ = false;
        error = "encoder batch deadline expired while sending requests";
        return std::nullopt;
      }
      const auto motor_id = motor_ids[index];
      if (!transport_->send(make_encoder_request(motor_id), error))
      {
        synchronized_ = false;
        error = "failed to request encoder from motor " + std::to_string(motor_id) + ": " + error;
        return std::nullopt;
      }
      if (request_spacing.count() > 0 && index + 1U < window_end)
      {
        std::this_thread::sleep_for(request_spacing);
      }
    }

    std::size_t window_received = 0U;
    while (window_received < window_end - window_begin)
    {
      const auto now = std::chrono::steady_clock::now();
      const bool deadline_expired = now >= deadline;
      if (deadline_expired && post_deadline_frames >= kMaxPostDeadlineFrames)
      {
        break;
      }
      const auto remaining = deadline_expired ? std::chrono::microseconds{0} :
        std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
      std::string receive_error;
      auto frame = transport_->receive(remaining, receive_error);
      if (!receive_error.empty())
      {
        synchronized_ = false;
        error = receive_error;
        return std::nullopt;
      }
      if (!frame)
      {
        if (deadline_expired)
        {
          break;
        }
        continue;
      }
      if (deadline_expired)
      {
        ++post_deadline_frames;
        ++counters_.post_deadline_drained;
      }

      const auto window_first = motor_ids.begin() + static_cast<std::ptrdiff_t>(window_begin);
      const auto window_last = motor_ids.begin() + static_cast<std::ptrdiff_t>(window_end);
      const auto motor = std::find(window_first, window_last, frame->id);
      const bool expected_encoder =
        motor != window_last && !frame->data.empty() &&
        frame->data.front() == kReadEncoderCommand;
      if (!expected_encoder)
      {
        classify_ignored(*frame);
        continue;
      }
      if (!has_valid_checksum(*frame))
      {
        ++counters_.bad_checksum;
        continue;
      }

      const auto index = static_cast<std::size_t>(std::distance(motor_ids.begin(), motor));
      const auto ticks = parse_encoder_response(*frame, *motor);
      if (!ticks)
      {
        ++counters_.malformed_encoder;
        continue;
      }
      if (pending_samples[index])
      {
        ++counters_.duplicate_encoder;
        continue;
      }
      pending_samples[index] = EncoderSample{*motor, *ticks, std::chrono::steady_clock::now()};
      ++window_received;
    }

    if (window_received != window_end - window_begin)
    {
      synchronized_ = false;
      error = "timed out waiting for encoder batch; missing CAN IDs:";
      for (std::size_t index = window_begin; index < window_end; ++index)
      {
        if (!pending_samples[index])
        {
          error += " " + std::to_string(motor_ids[index]);
        }
      }
      return std::nullopt;
    }
  }

  std::vector<EncoderSample> samples;
  samples.reserve(motor_ids.size());
  for (auto & sample : pending_samples)
  {
    samples.push_back(std::move(*sample));
  }
  error.clear();
  return samples;
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
