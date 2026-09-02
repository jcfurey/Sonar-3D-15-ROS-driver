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
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace sonar3d
{
namespace
{

[[noreturn]] void close_and_throw(int socket, const char * operation)
{
  const auto error = errno;
  close(socket);
  throw std::system_error(error, std::generic_category(), operation);
}

[[nodiscard]] in_addr parse_ipv4(const std::string & value, const char * parameter_name, int socket)
{
  in_addr address{};
  if (inet_pton(AF_INET, value.c_str(), &address) != 1) {
    close(socket);
    throw std::invalid_argument(std::string(parameter_name) + " is not a valid IPv4 address: " +
          value);
  }
  return address;
}

}  // namespace

UdpReceiver::UdpReceiver(
  const std::string & multicast_group,
  std::uint16_t port,
  const std::string & interface_address)
{
  if (port == 0) {
    throw std::invalid_argument("multicast_port must be greater than zero");
  }

  socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
  if (socket_ < 0) {
    throw std::system_error(errno, std::generic_category(), "create UDP socket");
  }

  const auto group = parse_ipv4(multicast_group, "multicast_group", socket_);
  if (!IN_MULTICAST(ntohl(group.s_addr))) {
    close(socket_);
    socket_ = -1;
    throw std::invalid_argument("multicast_group is not an IPv4 multicast address");
  }
  const auto interface = parse_ipv4(interface_address, "multicast_interface", socket_);

  const int reuse_address = 1;
  if (setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) < 0) {
    close_and_throw(socket_, "enable SO_REUSEADDR");
  }

  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(port);
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_, reinterpret_cast<const sockaddr *>(&bind_address), sizeof(bind_address)) < 0) {
    close_and_throw(socket_, "bind multicast UDP socket");
  }

  ip_mreq membership{};
  membership.imr_multiaddr = group;
  membership.imr_interface = interface;
  if (setsockopt(socket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &membership, sizeof(membership)) < 0) {
    close_and_throw(socket_, "join multicast group");
  }

  const auto current_flags = fcntl(socket_, F_GETFL, 0);
  if (current_flags < 0 || fcntl(socket_, F_SETFL, current_flags | O_NONBLOCK) < 0) {
    close_and_throw(socket_, "make multicast UDP socket nonblocking");
  }
}

UdpReceiver::~UdpReceiver()
{
  if (socket_ >= 0) {
    close(socket_);
  }
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

  std::array<char, INET_ADDRSTRLEN> source_text{};
  if (inet_ntop(AF_INET, &source.sin_addr, source_text.data(), source_text.size()) == nullptr) {
    throw std::system_error(errno, std::generic_category(), "format UDP source address");
  }

  Datagram datagram;
  datagram.bytes.assign(buffer_.begin(), buffer_.begin() + static_cast<std::size_t>(received));
  datagram.source_address = source_text.data();
  return datagram;
}

}  // namespace sonar3d
