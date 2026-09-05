// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

#include "sonar3d/udp_receiver.hpp"
#include "stream_test_support.hpp"

namespace
{

TEST(UdpReceiver, NonblockingReceivePreservesPacketOwnershipAndSource)
{
  const auto port = sonar3d::testing::unused_udp_port();
  sonar3d::UdpReceiver receiver("239.255.96.15", port, "127.0.0.1", 0);
  EXPECT_GT(receiver.receive_buffer_size(), 0);
  EXPECT_FALSE(receiver.receive());
  const sonar3d::testing::UdpSender sender;
  const auto first = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(1));
  const auto second = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(2, true));
  sender.send(port, first);
  const auto received_first = receiver.receive();
  ASSERT_TRUE(received_first);
  sender.send(port, second);
  const auto received_second = receiver.receive();
  ASSERT_TRUE(received_second);
  EXPECT_EQ(received_first->bytes, first);
  EXPECT_EQ(received_second->bytes, second);
  EXPECT_EQ(received_first->source_address, "127.0.0.1");
  EXPECT_FALSE(receiver.receive());
}

TEST(UdpReceiver, DefaultBufferRetainsABurstOfFullResolutionRangePackets)
{
  const auto port = sonar3d::testing::unused_udp_port();
  sonar3d::UdpReceiver receiver("239.255.96.15", port, "127.0.0.1");
  // Linux accounts for socket overhead as well as payload. A host with a low
  // rmem_max can clamp the request; report that constraint instead of assuming
  // the process can change a global network setting.
  if (receiver.receive_buffer_size() < 1024 * 1024) {
    GTEST_SKIP() << "OS receive-buffer limit is too low for this burst: " <<
      receiver.receive_buffer_size();
  }
  const sonar3d::testing::UdpSender sender;
  constexpr std::uint32_t kPackets = 16;
  for (std::uint32_t sequence = 0; sequence < kPackets; ++sequence) {
    sender.send(port, sonar3d::protocol::encode_packet(sonar3d::testing::range_image(sequence)));
  }
  for (std::uint32_t sequence = 0; sequence < kPackets; ++sequence) {
    const auto received = receiver.receive();
    ASSERT_TRUE(received) << "Missing packet " << sequence;
    const auto decoded = sonar3d::protocol::decode_packet(received->bytes);
    EXPECT_EQ(std::get<sonar3d::protocol::RangeImage>(decoded.message).header.sequence_id,
        sequence);
  }
  EXPECT_FALSE(receiver.receive());
}

TEST(UdpReceiver, RejectsNegativeBufferRequests)
{
  EXPECT_THROW(
    sonar3d::UdpReceiver("239.255.96.15", 4747, "127.0.0.1", -1), std::invalid_argument);
}

}  // namespace
