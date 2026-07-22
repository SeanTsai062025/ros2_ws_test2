#include "dexter_hardware/can_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace dexter_hardware
{

SocketCanTransport::SocketCanTransport(const std::string & interface_name)
{
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0)
  {
    throw std::runtime_error("socket(PF_CAN) failed: " + std::string(std::strerror(errno)));
  }

  struct ifreq request {};
  if (interface_name.size() >= IFNAMSIZ)
  {
    close();
    throw std::runtime_error("SocketCAN interface name is too long: " + interface_name);
  }
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0)
  {
    const std::string error = std::strerror(errno);
    close();
    throw std::runtime_error("Cannot resolve SocketCAN interface '" + interface_name + "': " + error);
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0)
  {
    const std::string error = std::strerror(errno);
    close();
    throw std::runtime_error("Cannot bind SocketCAN interface '" + interface_name + "': " + error);
  }

  struct can_filter filters[6] {};
  for (std::uint32_t index = 0; index < 6U; ++index)
  {
    filters[index].can_id = index + 1U;
    filters[index].can_mask = CAN_SFF_MASK;
  }
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0)
  {
    const std::string error = std::strerror(errno);
    close();
    throw std::runtime_error("Cannot set SocketCAN motor filters: " + error);
  }
}

SocketCanTransport::~SocketCanTransport()
{
  close();
}

bool SocketCanTransport::send(const CanFrame & frame, std::string & error)
{
  if (socket_fd_ < 0 || frame.id > CAN_SFF_MASK || frame.data.size() > CAN_MAX_DLEN)
  {
    error = "invalid SocketCAN send request";
    return false;
  }

  struct can_frame raw {};
  raw.can_id = frame.id;
  raw.can_dlc = static_cast<__u8>(frame.data.size());
  std::copy(frame.data.begin(), frame.data.end(), raw.data);
  const auto written = ::write(socket_fd_, &raw, sizeof(raw));
  if (written != static_cast<ssize_t>(sizeof(raw)))
  {
    error = "SocketCAN write failed: " + std::string(std::strerror(errno));
    return false;
  }
  error.clear();
  return true;
}

std::optional<CanFrame> SocketCanTransport::receive(
  const std::chrono::microseconds timeout, std::string & error)
{
  if (socket_fd_ < 0)
  {
    error = "SocketCAN transport is closed";
    return std::nullopt;
  }

  struct pollfd descriptor {socket_fd_, POLLIN, 0};
  const auto timeout_ms = static_cast<int>(std::max<std::int64_t>(
    0, (timeout.count() + 999) / 1000));
  const int poll_result = ::poll(&descriptor, 1, timeout_ms);
  if (poll_result == 0)
  {
    error.clear();
    return std::nullopt;
  }
  if (poll_result < 0)
  {
    if (errno == EINTR)
    {
      error.clear();
      return std::nullopt;
    }
    error = "SocketCAN poll failed: " + std::string(std::strerror(errno));
    return std::nullopt;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
  {
    error = "SocketCAN poll reported a bus/socket error";
    return std::nullopt;
  }

  struct can_frame raw {};
  const auto received = ::read(socket_fd_, &raw, sizeof(raw));
  if (received != static_cast<ssize_t>(sizeof(raw)))
  {
    error = "SocketCAN read failed: " + std::string(std::strerror(errno));
    return std::nullopt;
  }

  CanFrame frame;
  frame.id = raw.can_id & CAN_SFF_MASK;
  frame.data.assign(raw.data, raw.data + raw.can_dlc);
  error.clear();
  return frame;
}

void SocketCanTransport::close() noexcept
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

std::unique_ptr<CanTransport> make_socketcan_transport(const std::string & interface_name)
{
  return std::make_unique<SocketCanTransport>(interface_name);
}

}  // namespace dexter_hardware
