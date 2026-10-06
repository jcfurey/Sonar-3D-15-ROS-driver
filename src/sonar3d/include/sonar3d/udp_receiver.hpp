// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "sonar3d/protocol.hpp"

namespace sonar3d
{

struct Datagram
{
  std::vector<std::uint8_t> bytes;
  // IPv4 source address in host byte order.
  std::uint32_t source_address{};
};

// Parse a literal dotted-quad IPv4 address into host byte order.
[[nodiscard]] std::optional<std::uint32_t> parse_ipv4(const std::string & text);
[[nodiscard]] std::string format_ipv4(std::uint32_t address);

struct UdpReceiverOptions
{
  std::uint16_t port{4747};
  // Multicast: the interface that joins the group. Unicast: the local address
  // to bind; 0.0.0.0 accepts the port on every interface.
  std::string interface_address{"0.0.0.0"};
  // Empty receives unicast datagrams instead of joining a group.
  std::string multicast_group{"224.0.0.96"};
  // Zero keeps the OS default.
  int receive_buffer_size{1024 * 1024};
};

class UdpReceiver
{
public:
  explicit UdpReceiver(const UdpReceiverOptions & options);
  UdpReceiver(
    const std::string & multicast_group, std::uint16_t port,
    const std::string & interface_address, int receive_buffer_size = 1024 * 1024);
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver &) = delete;
  UdpReceiver & operator=(const UdpReceiver &) = delete;
  UdpReceiver(UdpReceiver &&) = delete;
  UdpReceiver & operator=(UdpReceiver &&) = delete;

  // Nonblocking: returns nullopt when no datagram is queued.
  [[nodiscard]] std::optional<Datagram> receive();

  // Block until a datagram is queued (true), the timeout expires, or
  // interrupt() is called (false). A negative timeout waits indefinitely.
  [[nodiscard]] bool wait(std::chrono::milliseconds timeout);

  // Wake every current and future wait() until clear_interrupt(). Safe to call
  // from any thread.
  void interrupt();
  void clear_interrupt();

  // Drop every queued datagram, returning how many were dropped.
  std::size_t discard_pending();

  [[nodiscard]] int receive_buffer_size() const {return receive_buffer_size_;}

private:
  int socket_{-1};
  int wake_event_{-1};
  int receive_buffer_size_{};
  std::array<std::uint8_t, protocol::kMaximumUdpPacketSize> buffer_{};
};

}  // namespace sonar3d
