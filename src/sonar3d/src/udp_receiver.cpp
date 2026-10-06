// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "sonar3d/udp_receiver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace sonar3d
{
namespace
{

void close_descriptors(int socket, int wake_event)
{
  if (socket >= 0) {
    close(socket);
  }
  if (wake_event >= 0) {
    close(wake_event);
  }
}

[[noreturn]] void close_and_throw(int socket, int wake_event, const char * operation)
{
  const auto error = errno;
  close_descriptors(socket, wake_event);
  throw std::system_error(error, std::generic_category(), operation);
}

[[nodiscard]] in_addr parse_ipv4_parameter(
  const std::string & value, const char * parameter_name,
  int socket, int wake_event)
{
  in_addr address{};
  if (inet_pton(AF_INET, value.c_str(), &address) != 1) {
    close_descriptors(socket, wake_event);
    throw std::invalid_argument(std::string(parameter_name) + " is not a valid IPv4 address: " +
          value);
  }
  return address;
}

}  // namespace

std::optional<std::uint32_t> parse_ipv4(const std::string & text)
{
  in_addr address{};
  if (inet_pton(AF_INET, text.c_str(), &address) != 1) {
    return std::nullopt;
  }
  return ntohl(address.s_addr);
}

std::string format_ipv4(std::uint32_t address)
{
  in_addr value{};
  value.s_addr = htonl(address);
  std::array<char, INET_ADDRSTRLEN> text{};
  if (inet_ntop(AF_INET, &value, text.data(), text.size()) == nullptr) {
    throw std::system_error(errno, std::generic_category(), "format IPv4 address");
  }
  return text.data();
}

UdpReceiver::UdpReceiver(
  const std::string & multicast_group,
  std::uint16_t port,
  const std::string & interface_address,
  int receive_buffer_size)
{
  if (port == 0) {
    throw std::invalid_argument("multicast_port must be greater than zero");
  }
  if (receive_buffer_size < 0) {
    throw std::invalid_argument("receive_buffer_size must not be negative");
  }

  socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, IPPROTO_UDP);
  if (socket_ < 0) {
    throw std::system_error(errno, std::generic_category(), "create UDP socket");
  }
  wake_event_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (wake_event_ < 0) {
    close_and_throw(socket_, -1, "create receiver wake event");
  }

  const auto group = parse_ipv4_parameter(multicast_group, "multicast_group", socket_,
      wake_event_);
  if (!IN_MULTICAST(ntohl(group.s_addr))) {
    close_descriptors(socket_, wake_event_);
    throw std::invalid_argument("multicast_group is not an IPv4 multicast address");
  }
  const auto interface = parse_ipv4_parameter(
    interface_address, "multicast_interface", socket_, wake_event_);

  const int reuse_address = 1;
  if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) < 0) {
    close_and_throw(socket_, wake_event_, "enable SO_REUSEADDR");
  }
  if (receive_buffer_size > 0 && setsockopt(
      socket_, SOL_SOCKET, SO_RCVBUF, &receive_buffer_size, sizeof(receive_buffer_size)) < 0)
  {
    close_and_throw(socket_, wake_event_, "set UDP receive buffer size");
  }
  socklen_t buffer_size_length = sizeof(receive_buffer_size_);
  if (getsockopt(
      socket_, SOL_SOCKET, SO_RCVBUF, &receive_buffer_size_, &buffer_size_length) < 0)
  {
    close_and_throw(socket_, wake_event_, "read UDP receive buffer size");
  }

  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(port);
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_, reinterpret_cast<const sockaddr *>(&bind_address), sizeof(bind_address)) < 0) {
    close_and_throw(socket_, wake_event_, "bind multicast UDP socket");
  }

  ip_mreq membership{};
  membership.imr_multiaddr = group;
  membership.imr_interface = interface;
  if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &membership, sizeof(membership)) < 0) {
    close_and_throw(socket_, wake_event_, "join multicast group");
  }
}

UdpReceiver::~UdpReceiver()
{
  close_descriptors(socket_, wake_event_);
}

std::optional<Datagram> UdpReceiver::receive()
{
  sockaddr_in source{};
  socklen_t source_size = sizeof(source);
  ssize_t received{};
  do {
    received = recvfrom(
      socket_,
      buffer_.data(),
      buffer_.size(),
      0,
      reinterpret_cast<sockaddr *>(&source),
      &source_size);
  } while (received < 0 && errno == EINTR);

  if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return std::nullopt;
  }
  if (received < 0) {
    throw std::system_error(errno, std::generic_category(), "receive multicast UDP packet");
  }

  Datagram datagram;
  datagram.bytes.assign(buffer_.begin(), buffer_.begin() + static_cast<std::size_t>(received));
  datagram.source_address = ntohl(source.sin_addr.s_addr);
  return datagram;
}

bool UdpReceiver::wait(std::chrono::milliseconds timeout)
{
  std::array<pollfd, 2> descriptors{{
    {socket_, POLLIN, 0},
    {wake_event_, POLLIN, 0},
  }};
  const auto timeout_ms = timeout.count() < 0 ? -1 : static_cast<int>(
    std::min<std::chrono::milliseconds::rep>(timeout.count(), std::numeric_limits<int>::max()));
  int ready{};
  do {
    ready = poll(descriptors.data(), descriptors.size(), timeout_ms);
  } while (ready < 0 && errno == EINTR);
  if (ready < 0) {
    throw std::system_error(errno, std::generic_category(), "wait for multicast UDP packet");
  }
  // The wake event is never drained, so an interrupt is permanent.
  if (descriptors[1].revents != 0) {
    return false;
  }
  return (descriptors[0].revents & (POLLIN | POLLERR)) != 0;
}

void UdpReceiver::interrupt()
{
  const std::uint64_t increment = 1;
  // EAGAIN only occurs when the counter is saturated, which still wakes waiters.
  [[maybe_unused]] const auto written = write(wake_event_, &increment, sizeof(increment));
}

void UdpReceiver::clear_interrupt()
{
  // Reading an eventfd returns and resets its counter; EAGAIN means it was clear.
  std::uint64_t counter{};
  [[maybe_unused]] const auto read_bytes = read(wake_event_, &counter, sizeof(counter));
}

std::size_t UdpReceiver::discard_pending()
{
  std::size_t discarded{};
  while (true) {
    if (recv(socket_, buffer_.data(), buffer_.size(), MSG_DONTWAIT) >= 0) {
      ++discarded;
    } else if (errno != EINTR) {
      return discarded;
    }
  }
}

}  // namespace sonar3d
