#ifndef DEXTER_HARDWARE__CAN_TRANSPORT_HPP_
#define DEXTER_HARDWARE__CAN_TRANSPORT_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "dexter_hardware/mks_protocol.hpp"

namespace dexter_hardware
{

class CanTransport
{
public:
  virtual ~CanTransport() = default;
  virtual bool send(const CanFrame & frame, std::string & error) = 0;
  virtual std::optional<CanFrame> receive(
    std::chrono::microseconds timeout, std::string & error) = 0;
  virtual void close() noexcept = 0;
};

class SocketCanTransport final : public CanTransport
{
public:
  explicit SocketCanTransport(const std::string & interface_name);
  ~SocketCanTransport() override;

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  bool send(const CanFrame & frame, std::string & error) override;
  std::optional<CanFrame> receive(
    std::chrono::microseconds timeout, std::string & error) override;
  void close() noexcept override;

private:
  int socket_fd_{-1};
};

std::unique_ptr<CanTransport> make_socketcan_transport(const std::string & interface_name);

}  // namespace dexter_hardware

#endif  // DEXTER_HARDWARE__CAN_TRANSPORT_HPP_
