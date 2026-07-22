#ifndef DEXTER_HARDWARE__MKS_CAN_CLIENT_HPP_
#define DEXTER_HARDWARE__MKS_CAN_CLIENT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "dexter_hardware/can_transport.hpp"

namespace dexter_hardware
{

struct ReceiveCounters
{
  std::size_t drained{0};
  std::size_t f5_status{0};
  std::size_t unrelated{0};
  std::size_t bad_checksum{0};
};

class MksCanClient
{
public:
  explicit MksCanClient(std::unique_ptr<CanTransport> transport);
  ~MksCanClient();

  bool resynchronize(
    std::chrono::microseconds quiet_period, std::chrono::microseconds max_wait,
    std::string & error);
  std::optional<std::int64_t> read_encoder(
    std::uint32_t motor_id, std::chrono::microseconds timeout, std::string & error);
  std::optional<std::uint32_t> read_parameter(
    std::uint32_t motor_id, std::uint8_t parameter,
    std::chrono::milliseconds timeout, std::string & error);
  bool set_parameter(
    std::uint32_t motor_id, std::uint8_t parameter,
    const std::vector<std::uint8_t> & payload,
    std::chrono::milliseconds timeout, std::string & error);
  bool send_absolute(
    std::uint32_t motor_id, std::uint16_t speed, std::uint8_t acceleration,
    std::int32_t target_ticks, std::string & error);
  bool send_stop(std::uint32_t motor_id, std::string & error);
  void close() noexcept;

  const ReceiveCounters & counters() const { return counters_; }
  bool synchronized() const { return synchronized_; }

private:
  bool drain_pending(std::string & error);
  std::optional<CanFrame> wait_for_response(
    std::uint32_t motor_id, std::uint8_t command,
    std::chrono::microseconds timeout, std::string & error);
  void classify_ignored(const CanFrame & frame);

  std::unique_ptr<CanTransport> transport_;
  ReceiveCounters counters_;
  bool synchronized_{false};
};

}  // namespace dexter_hardware

#endif  // DEXTER_HARDWARE__MKS_CAN_CLIENT_HPP_
