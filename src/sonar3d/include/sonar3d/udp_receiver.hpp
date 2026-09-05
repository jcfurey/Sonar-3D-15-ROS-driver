// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
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
  std::string source_address;
};

class UdpReceiver
{
public:
  UdpReceiver(
    const std::string & multicast_group, std::uint16_t port,
    const std::string & interface_address, int receive_buffer_size = 1024 * 1024);
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver &) = delete;
  UdpReceiver & operator=(const UdpReceiver &) = delete;
  UdpReceiver(UdpReceiver &&) = delete;
  UdpReceiver & operator=(UdpReceiver &&) = delete;

  [[nodiscard]] std::optional<Datagram> receive();
  [[nodiscard]] int receive_buffer_size() const {return receive_buffer_size_;}

private:
  int socket_{-1};
  int receive_buffer_size_{};
  std::array<std::uint8_t, protocol::kMaximumUdpPacketSize> buffer_{};
};

}  // namespace sonar3d
