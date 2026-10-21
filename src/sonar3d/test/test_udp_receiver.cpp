// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <pthread.h>

#include <chrono>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <future>
#include <stdexcept>
#include <system_error>
#include <thread>

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
  EXPECT_EQ(sonar3d::format_ipv4(received_first->source_address), "127.0.0.1");
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

TEST(UdpReceiver, WaitWakesForDataAndInterruptsPermanently)
{
  using namespace std::chrono_literals;
  const auto port = sonar3d::testing::unused_udp_port();
  sonar3d::UdpReceiver receiver("239.255.96.15", port, "127.0.0.1");
  EXPECT_FALSE(receiver.wait(1ms));
  const sonar3d::testing::UdpSender sender;
  sender.send(port, sonar3d::protocol::encode_packet(sonar3d::testing::range_image(1)));
  EXPECT_TRUE(receiver.wait(1s));
  EXPECT_TRUE(receiver.receive());

  std::thread waiter([&receiver] {EXPECT_FALSE(receiver.wait(-1ms));});
  std::this_thread::sleep_for(10ms);
  receiver.interrupt();
  waiter.join();
  EXPECT_FALSE(receiver.wait(-1ms));
}

TEST(UdpReceiver, ParsesAndFormatsIpv4InHostOrder)
{
  EXPECT_EQ(sonar3d::parse_ipv4("192.168.194.96"), 0xC0A8C260U);
  EXPECT_EQ(sonar3d::format_ipv4(0xC0A8C260U), "192.168.194.96");
  EXPECT_FALSE(sonar3d::parse_ipv4("sonar.local"));
  EXPECT_FALSE(sonar3d::parse_ipv4(""));
}

TEST(UdpReceiver, UnicastReceivesOnItsAddressAndDoesNotSharePorts)
{
  const auto port = sonar3d::testing::unused_udp_port();
  sonar3d::UdpReceiver receiver(sonar3d::UdpReceiverOptions{port, "127.0.0.1", "", 0});
  const sonar3d::testing::UdpSender sender;
  const auto packet = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(3));
  sender.send(port, packet, "127.0.0.1");
  ASSERT_TRUE(receiver.wait(std::chrono::seconds(1)));
  const auto received = receiver.receive();
  ASSERT_TRUE(received);
  EXPECT_EQ(received->bytes, packet);
  EXPECT_EQ(sonar3d::format_ipv4(received->source_address), "127.0.0.1");
  // A second unicast listener would silently take the stream; refuse it.
  EXPECT_THROW(
    sonar3d::UdpReceiver(sonar3d::UdpReceiverOptions{port, "127.0.0.1", "", 0}),
    std::system_error);
}

TEST(UdpReceiver, RejectsNegativeBufferRequests)
{
  EXPECT_THROW(
    sonar3d::UdpReceiver("239.255.96.15", 4747, "127.0.0.1", -1), std::invalid_argument);
}

TEST(UdpReceiver, MulticastListenersOnOnePortReceiveOnlyTheirOwnGroup)
{
  const auto port = sonar3d::testing::unused_udp_port();
  sonar3d::UdpReceiver first("239.255.96.15", port, "127.0.0.1");
  sonar3d::UdpReceiver second("239.255.96.16", port, "127.0.0.1");
  const sonar3d::testing::UdpSender sender;
  const auto a = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(1));
  const auto b = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(2));
  sender.send(port, a, "239.255.96.15");
  sender.send(port, b, "239.255.96.16");
  ASSERT_TRUE(first.wait(std::chrono::seconds(1)));
  ASSERT_TRUE(second.wait(std::chrono::seconds(1)));
  const auto received_a = first.receive();
  const auto received_b = second.receive();
  ASSERT_TRUE(received_a);
  ASSERT_TRUE(received_b);
  EXPECT_EQ(received_a->bytes, a);
  EXPECT_EQ(received_b->bytes, b);
  EXPECT_FALSE(first.receive());
  EXPECT_FALSE(second.receive());
}

TEST(UdpReceiver, SignalsDoNotRestartAFiniteWait)
{
  using namespace std::chrono_literals;
  const auto port = sonar3d::testing::unused_udp_port();
  sonar3d::UdpReceiver receiver("239.255.96.15", port, "127.0.0.1");
  struct sigaction handler{};
  struct sigaction previous{};
  handler.sa_handler = [](int) {};
  sigemptyset(&handler.sa_mask);
  ASSERT_EQ(sigaction(SIGUSR1, &handler, &previous), 0);
  std::promise<void> started;
  auto running = started.get_future();
  std::atomic<bool> finished{false};
  bool ready = true;
  std::thread waiter([&] {
      started.set_value();
      ready = receiver.wait(25ms);
      finished = true;
    });
  running.wait();
  // Keep interrupting much longer than the requested timeout. A finite wait
  // must still finish while signals continue to arrive.
  for (unsigned count = 0; count < 100 && !finished; ++count) {
    pthread_kill(waiter.native_handle(), SIGUSR1);
    std::this_thread::sleep_for(3ms);
  }
  const bool finished_during_signals = finished;
  waiter.join();
  sigaction(SIGUSR1, &previous, nullptr);
  EXPECT_FALSE(ready);
  EXPECT_TRUE(finished_during_signals);
}

}  // namespace
