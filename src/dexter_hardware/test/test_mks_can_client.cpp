#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
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
    if (delay_next_nonempty_receive)
    {
      delay_next_nonempty_receive = false;
      std::this_thread::sleep_for(nonempty_receive_delay);
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
  bool delay_next_nonempty_receive{false};
  std::chrono::microseconds nonempty_receive_delay{0};
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

CanFrame malformed_encoder_response(const std::uint32_t id)
{
  CanFrame frame{id, {kReadEncoderCommand, 0x01U}};
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
  const auto ticks = client->read_encoder(2U, std::chrono::milliseconds{5}, error);
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
  const auto first = client->read_encoder(1U, std::chrono::milliseconds{5}, error);
  const auto second = client->read_encoder(1U, std::chrono::milliseconds{5}, error);
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
  EXPECT_FALSE(client->read_encoder(4U, std::chrono::milliseconds{1}, error));
  EXPECT_FALSE(client->synchronized());

  transport->pending.push_back(encoder_response(4U, 400));
  EXPECT_FALSE(client->read_encoder(4U, std::chrono::milliseconds{1}, error));

  EXPECT_TRUE(client->resynchronize(
    std::chrono::microseconds{1}, std::chrono::microseconds{100}, error));
  transport->on_send = [](const CanFrame &, std::deque<CanFrame> & responses) {
      responses.push_back(encoder_response(4U, 401));
    };
  const auto recovered = client->read_encoder(4U, std::chrono::milliseconds{5}, error);
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
  const auto ticks = client->read_encoder(3U, std::chrono::milliseconds{5}, error);
  ASSERT_TRUE(ticks);
  EXPECT_EQ(*ticks, 300);
  EXPECT_EQ(client->counters().unrelated, 1U);
}

TEST(MksCanClient, BatchesRequestsAndReturnsValidatedSamplesInRequestedOrder)
{
  auto [client, transport] = make_client();
  transport->on_send = [transport](const CanFrame &, std::deque<CanFrame> & responses) {
      // Responses only become available after all six requests were sent. A client
      // that serializes request/response transactions cannot pass this test.
      if (transport->sent.size() != 6U)
      {
        return;
      }
      responses.push_back(f5_status(4U, 1U));
      responses.push_back(encoder_response(6U, -600));
      responses.push_back(encoder_response(2U, 999, false));
      responses.push_back(encoder_response(4U, 400));
      responses.push_back(malformed_encoder_response(3U));
      responses.push_back(encoder_response(2U, 200));
      responses.push_back(encoder_response(2U, 201));
      responses.push_back(encoder_response(5U, 500));
      responses.push_back(encoder_response(1U, 100));
      responses.push_back(encoder_response(3U, 300));
    };

  std::string error;
  const std::vector<std::uint32_t> ids{1U, 2U, 3U, 4U, 5U, 6U};
  const auto samples = client->read_encoders(ids, std::chrono::milliseconds{5}, error);
  ASSERT_TRUE(samples) << error;
  ASSERT_EQ(samples->size(), ids.size());
  const std::array<std::int64_t, 6> expected_ticks{100, 200, 300, 400, 500, -600};
  for (std::size_t index = 0; index < ids.size(); ++index)
  {
    EXPECT_EQ((*samples)[index].motor_id, ids[index]);
    EXPECT_EQ((*samples)[index].ticks, expected_ticks[index]);
  }
  EXPECT_EQ(transport->sent.size(), 6U);
  EXPECT_EQ(client->counters().f5_status, 1U);
  EXPECT_EQ(client->counters().bad_checksum, 1U);
  EXPECT_EQ(client->counters().malformed_encoder, 1U);
  EXPECT_EQ(client->counters().duplicate_encoder, 1U);
}

TEST(MksCanClient, IncompleteBatchLatchesUnsynchronizedAndNamesMissingMotors)
{
  auto [client, transport] = make_client();
  transport->on_send = [transport](const CanFrame &, std::deque<CanFrame> & responses) {
      if (transport->sent.size() == 6U)
      {
        for (std::uint32_t id = 1U; id <= 5U; ++id)
        {
          responses.push_back(encoder_response(id, static_cast<std::int64_t>(id * 10U)));
        }
      }
    };

  std::string error;
  const std::vector<std::uint32_t> ids{1U, 2U, 3U, 4U, 5U, 6U};
  EXPECT_FALSE(client->read_encoders(ids, std::chrono::milliseconds{1}, error));
  EXPECT_NE(error.find("missing CAN IDs: 6"), std::string::npos);
  EXPECT_FALSE(client->synchronized());

  transport->pending.push_back(encoder_response(6U, 60));
  EXPECT_FALSE(client->read_encoders(ids, std::chrono::milliseconds{1}, error));
  EXPECT_EQ(transport->sent.size(), 6U);
}

TEST(MksCanClient, DrainsKernelQueuedBatchAfterUserspaceSchedulingPause)
{
  auto [client, transport] = make_client();
  transport->on_send = [transport](const CanFrame &, std::deque<CanFrame> & responses) {
      if (transport->sent.size() == 6U)
      {
        for (std::uint32_t id = 1U; id <= 6U; ++id)
        {
          responses.push_back(encoder_response(id, static_cast<std::int64_t>(id * 100U)));
        }
      }
    };
  transport->nonempty_receive_delay = std::chrono::milliseconds{2};
  transport->delay_next_nonempty_receive = true;

  std::string error;
  const std::vector<std::uint32_t> ids{1U, 2U, 3U, 4U, 5U, 6U};
  const auto samples = client->read_encoders(ids, std::chrono::milliseconds{1}, error);
  ASSERT_TRUE(samples) << error;
  ASSERT_EQ(samples->size(), ids.size());
  EXPECT_TRUE(client->synchronized());
  EXPECT_GE(client->counters().post_deadline_drained, 5U);
}

TEST(MksCanClient, RejectsDuplicateMotorIdsBeforeTransmitting)
{
  auto [client, transport] = make_client();
  std::string error;
  EXPECT_FALSE(client->read_encoders({1U, 2U, 1U}, std::chrono::milliseconds{1}, error));
  EXPECT_NE(error.find("duplicate motor ID 1"), std::string::npos);
  EXPECT_TRUE(client->synchronized());
  EXPECT_TRUE(transport->sent.empty());
}

}  // namespace
}  // namespace dexter_hardware
