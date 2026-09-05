// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <span>

#include <cerrno>
#include <cstdint>
#include <system_error>

#include "sonar3d/protocol.hpp"

namespace sonar3d::testing
{

class UdpSender
{
public:
  UdpSender()
  : socket_(socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP))
  {
    if (socket_ < 0) {
      throw std::system_error(errno, std::generic_category(), "create test UDP socket");
    }
    in_addr interface{};
    interface.s_addr = htonl(INADDR_LOOPBACK);
    if (setsockopt(socket_, IPPROTO_IP, IP_MULTICAST_IF, &interface, sizeof(interface)) < 0) {
      const auto error = errno;
      close(socket_);
      throw std::system_error(error, std::generic_category(), "select test multicast interface");
    }
  }
  ~UdpSender() {close(socket_);}
  UdpSender(const UdpSender &) = delete;
  UdpSender & operator=(const UdpSender &) = delete;

  // Ask the OS for an unused port; the driver binds it immediately afterward.
  std::uint16_t unused_port() const
  {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::system_error(errno, std::generic_category(), "bind test UDP socket");
    }
    socklen_t size = sizeof(address);
    if (getsockname(socket_, reinterpret_cast<sockaddr *>(&address), &size) < 0) {
      throw std::system_error(errno, std::generic_category(), "read test UDP port");
    }
    return ntohs(address.sin_port);
  }

  void send(
    std::uint16_t port, std::span<const std::uint8_t> bytes,
    const char * destination = "127.0.0.1") const
  {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, destination, &address.sin_addr) != 1) {
      throw std::invalid_argument("invalid test UDP destination");
    }
    if (sendto(
        socket_, bytes.data(), bytes.size(), 0,
        reinterpret_cast<const sockaddr *>(&address), sizeof(address)) !=
      static_cast<ssize_t>(bytes.size()))
    {
      throw std::system_error(errno, std::generic_category(), "send test UDP packet");
    }
  }

private:
  int socket_;
};

inline std::uint16_t unused_udp_port()
{
  return UdpSender().unused_port();
}

inline protocol::Timestamp timestamp(std::int64_t nanoseconds)
{
  return {nanoseconds / 1'000'000'000, static_cast<std::int32_t>(nanoseconds % 1'000'000'000)};
}

inline protocol::RangeImage range_image(std::uint32_t sequence = 0, bool sparse = false)
{
  protocol::RangeImage image;
  image.header.timestamp = timestamp(1'700'000'000'000'000'000 + sequence * 50'000'000LL);
  image.header.sequence_id = sequence;
  image.width = 256;
  image.height = 64;
  image.horizontal_fov_degrees = 40.0F;
  image.vertical_fov_degrees = 40.0F;
  image.pixel_scale = 0.001F;
  std::uint32_t random = 42;
  for (std::size_t index = 0; index < image.width * image.height; ++index) {
    random = random * 1664525U + 1013904223U;
    image.pixels.push_back(sparse && index % 4 ? 0 : 1000 + (random >> 16) % 10000);
  }
  return image;
}

inline protocol::BitmapImage bitmap_image(std::uint32_t sequence, protocol::BitmapImageType type)
{
  protocol::BitmapImage image;
  image.header.timestamp = timestamp(1'700'000'000'000'000'000 + sequence * 50'000'000LL);
  image.header.sequence_id = sequence;
  image.width = 256;
  image.height = 64;
  image.type = type;
  image.pixels.resize(image.width * image.height);
  for (std::size_t index = 0; index < image.pixels.size(); ++index) {
    image.pixels[index] = static_cast<std::uint8_t>(index + sequence + static_cast<int>(type));
  }
  return image;
}

inline protocol::ImuBatch imu_batch(std::uint32_t sequence)
{
  protocol::ImuBatch batch;
  batch.sequence_id = sequence;
  batch.sample_count = 5;
  for (std::uint32_t sample = 0; sample < batch.sample_count; ++sample) {
    batch.timestamps.push_back(timestamp(
        1'700'000'000'000'000'000 + sequence * 50'000'000LL - 40'000'000 + sample * 10'000'000));
    batch.specific_force.insert(batch.specific_force.end(), {1.0F, 2.0F, 3.0F});
    batch.rate_of_turn.insert(batch.rate_of_turn.end(), {4.0F, 5.0F, 6.0F});
  }
  return batch;
}

}  // namespace sonar3d::testing
