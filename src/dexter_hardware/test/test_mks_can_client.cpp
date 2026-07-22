#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "dexter_hardware/mks_can_client.hpp"

namespace dexter_hardware
{
namespace
{

class FakeCanTransport final : public CanTransport
{
public:
  bool send(const CanFrame & frame, std::string & error) override
  {
    sent.push_back(frame);
    error.clear();
    if (on_send)
    {
      on_send(frame, pending);
    }
    return send_succeeds;
  }

  std::optional<CanFrame> receive(std::chrono::microseconds, std::string & error) override
  {
    error.clear();
    if (pending.empty())
    {
      return std::nullopt;
    }
    auto frame = pending.front();
    pending.pop_front();
    return frame;
  }

  void close() noexcept override { closed = true; }

  std::deque<CanFrame> pending;
  std::vector<CanFrame> sent;
  std::function<void(const CanFrame &, std::deque<CanFrame> &)> on_send;
  bool send_succeeds{true};
  bool closed{false};
};

CanFrame encoder_response(const std::uint32_t id, const std::int64_t ticks, bool valid = true)
{
  const auto raw = static_cast<std::uint64_t>(ticks) & 0x0000FFFFFFFFFFFFULL;
  CanFrame frame{id, {kReadEncoderCommand}};
  for (int shift = 40; shift >= 0; shift -= 8)
  {
    frame.data.push_back(static_cast<std::uint8_t>((raw >> shift) & 0xFFU));
  }
  frame.data.push_back(checksum(id, frame.data));
  if (!valid)
  {
    frame.data.back() ^= 0x80U;
  }
  return frame;
}

CanFrame f5_status(const std::uint32_t id, const std::uint8_t status)
{
  CanFrame frame{id, {kAbsoluteAxisCommand, status}};
  frame.data.push_back(checksum(id, frame.data));
  return frame;
}

std::pair<std::unique_ptr<MksCanClient>, FakeCanTransport *> make_client()
{
  auto transport = std::make_unique<FakeCanTransport>();
  auto * raw = transport.get();
  auto client = std::make_unique<MksCanClient>(std::move(transport));
  std::string error;
  EXPECT_TRUE(client->resynchronize(
    std::chrono::microseconds{1}, std::chrono::microseconds{100}, error));
  return {std::move(client), raw};
}

TEST(MksCanClient, DemultiplexesInterleavedStatusAndEncoderFrames)
{
  auto [client, transport] = make_client();
  transport->on_send = [](const CanFrame & request, std::deque<CanFrame> & responses) {
      if (request.data.front() == kReadEncoderCommand)
      {
        responses.push_back(f5_status(2U, 1U));
        responses.push_back(encoder_response(1U, 111));
        responses.push_back(encoder_response(2U, 222, false));
        responses.push_back(f5_status(6U, 2U));
        responses.push_back(encoder_response(2U, -123456789));
      }
    };
  std::string error;
  const auto ticks = client->read_encoder(2U, std::chrono::microseconds{100}, error);
  ASSERT_TRUE(ticks);
  EXPECT_EQ(*ticks, -123456789);
  EXPECT_EQ(client->counters().f5_status, 2U);
  EXPECT_EQ(client->counters().unrelated, 1U);
  EXPECT_EQ(client->counters().bad_checksum, 1U);
}

TEST(MksCanClient, DrainsStaleAndDuplicateResponsesBeforeEachRequest)
{
  auto [client, transport] = make_client();
  transport->pending.push_back(encoder_response(1U, 10));
  std::int64_t next_ticks = 20;
  transport->on_send = [&next_ticks](const CanFrame &, std::deque<CanFrame> & responses) {
      responses.push_back(encoder_response(1U, next_ticks));
      responses.push_back(encoder_response(1U, next_ticks));
      next_ticks += 10;
    };

  std::string error;
  const auto first = client->read_encoder(1U, std::chrono::microseconds{100}, error);
  const auto second = client->read_encoder(1U, std::chrono::microseconds{100}, error);
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_EQ(*first, 20);
  EXPECT_EQ(*second, 30);
  EXPECT_GE(client->counters().drained, 2U);
}

TEST(MksCanClient, MissingResponseLatchesUnsynchronizedAndRejectsDelayedFrame)
{
  auto [client, transport] = make_client();
  transport->on_send = nullptr;
  std::string error;
  EXPECT_FALSE(client->read_encoder(4U, std::chrono::microseconds{20}, error));
  EXPECT_FALSE(client->synchronized());

  transport->pending.push_back(encoder_response(4U, 400));
  EXPECT_FALSE(client->read_encoder(4U, std::chrono::microseconds{20}, error));

  EXPECT_TRUE(client->resynchronize(
    std::chrono::microseconds{1}, std::chrono::microseconds{100}, error));
  transport->on_send = [](const CanFrame &, std::deque<CanFrame> & responses) {
      responses.push_back(encoder_response(4U, 401));
    };
  const auto recovered = client->read_encoder(4U, std::chrono::microseconds{100}, error);
  ASSERT_TRUE(recovered);
  EXPECT_EQ(*recovered, 401);
}

TEST(MksCanClient, OutOfOrderResponseNeverSatisfiesCurrentMotorRequest)
{
  auto [client, transport] = make_client();
  transport->on_send = [](const CanFrame &, std::deque<CanFrame> & responses) {
      responses.push_back(encoder_response(6U, 600));
      responses.push_back(encoder_response(3U, 300));
    };
  std::string error;
  const auto ticks = client->read_encoder(3U, std::chrono::microseconds{100}, error);
  ASSERT_TRUE(ticks);
  EXPECT_EQ(*ticks, 300);
  EXPECT_EQ(client->counters().unrelated, 1U);
}

}  // namespace
}  // namespace dexter_hardware
