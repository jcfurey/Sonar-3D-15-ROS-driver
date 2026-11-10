// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <sys/resource.h>
#include <unistd.h>

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "sonar3d/native_recorder.hpp"
#include "sonar3d/protocol.hpp"

namespace
{

class NativeRecording : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/sonar3d-native-XXXXXX";
    const auto directory = mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    directory_ = directory;
  }
  void TearDown() override {std::filesystem::remove_all(directory_);}
  std::string path(const std::string & name = "capture.sonar") const
  {
    return (directory_ / name).string();
  }
  static std::vector<std::uint8_t> read(const std::string & name)
  {
    std::ifstream stream(name, std::ios::binary);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  }
  std::filesystem::path directory_;
};

TEST_F(NativeRecording, ExactOrderAndFlushPreserveEvenUndecodableDatagrams)
{
  sonar3d::NativeRecorder recorder;
  const std::vector<std::uint8_t> first{0, 255, 1, 42};
  const std::vector<std::uint8_t> second{'R', 'I', 'P', '2', 0};
  EXPECT_FALSE(recorder.enqueue(first));
  recorder.start(path());
  ASSERT_TRUE(recorder.enqueue(first));
  ASSERT_TRUE(recorder.enqueue(second));
  const auto result = recorder.stop();
  EXPECT_FALSE(result.active);
  EXPECT_EQ(result.packets, 2U);
  EXPECT_EQ(result.bytes, first.size() + second.size());
  EXPECT_EQ(result.queued_bytes, 0U);
  EXPECT_EQ(result.dropped_packets, 0U);
  EXPECT_TRUE(result.error.empty());
  auto expected = first;
  expected.insert(expected.end(), second.begin(), second.end());
  EXPECT_EQ(read(path()), expected);
  EXPECT_EQ(recorder.stop().packets, 2U);
}

TEST_F(NativeRecording, RejectsOverwriteAndInvalidPathsAndConcurrentStart)
{
  sonar3d::NativeRecorder recorder;
  EXPECT_THROW(recorder.start("relative.sonar"), std::invalid_argument);
  EXPECT_THROW(recorder.start(path("wrong.txt")), std::invalid_argument);
  EXPECT_THROW(recorder.start(path(), 0), std::invalid_argument);
  EXPECT_THROW(recorder.start(path("absent/capture.sonar")), std::system_error);
  recorder.start(path());
  EXPECT_THROW(recorder.start(path("other.sonar")), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path("other.sonar")));
  recorder.stop();
  EXPECT_THROW(recorder.start(path()), std::system_error);
  EXPECT_EQ(std::filesystem::file_size(path()), 0U);
}

TEST_F(NativeRecording, QueueOverflowEndsCaptureAndRemainsAnErrorUntilRestart)
{
  sonar3d::NativeRecorder recorder;
  recorder.start(path(), 4);
  EXPECT_FALSE(recorder.enqueue(std::vector<std::uint8_t>(5, 42)));
  const auto result = recorder.stop();
  EXPECT_FALSE(result.active);
  EXPECT_EQ(result.dropped_packets, 1U);
  EXPECT_EQ(result.bytes, 0U);
  EXPECT_NE(result.error.find("queue limit"), std::string::npos);
  EXPECT_EQ(recorder.stop().error, result.error);
  recorder.start(path("next.sonar"));
  EXPECT_TRUE(recorder.enqueue(std::vector<std::uint8_t>{42}));
  const auto next = recorder.stop();
  EXPECT_TRUE(next.error.empty());
  EXPECT_EQ(next.dropped_packets, 0U);
  EXPECT_EQ(next.packets, 1U);
}

TEST_F(NativeRecording, DiskWriteFailureMarksCaptureIncomplete)
{
  // Change the write limit only in an isolated child, before its writer starts.
  EXPECT_EXIT({
      rlimit limit;
      limit.rlim_cur = 4096;
      limit.rlim_max = 4096;
      if (setrlimit(RLIMIT_FSIZE, &limit) != 0) {_exit(2);}
      std::signal(SIGXFSZ, SIG_IGN);
      sonar3d::NativeRecorder recorder;
      recorder.start(path());
      if (!recorder.enqueue(std::vector<std::uint8_t>(8192, 42))) {_exit(3);}
      const auto result = recorder.stop();
      _exit(!result.active && result.bytes == 4096 && result.packets == 0 &&
      result.dropped_packets == 1 && result.queued_bytes == 0 &&
      result.error.find("write failed") != std::string::npos ? 0 : 4);
  }, ::testing::ExitedWithCode(0), "");
}

TEST_F(NativeRecording, DestructorDrainsAcceptedPackets)
{
  const std::vector<std::uint8_t> packet(65507, 42);
  {
    sonar3d::NativeRecorder recorder;
    recorder.start(path());
    for (int i = 0; i < 100; ++i) {
      ASSERT_TRUE(recorder.enqueue(packet));
    }
  }
  EXPECT_EQ(std::filesystem::file_size(path()), packet.size() * 100);
}

TEST_F(NativeRecording, OptionalVendorReferencesRoundTripByteForByte)
{
  const auto reference_directory = std::getenv("SONAR3D_REFERENCE_DIR");
  if (!reference_directory) {
    GTEST_SKIP() << "Set SONAR3D_REFERENCE_DIR to check vendor .sonar files";
  }
  std::size_t files = 0;
  for (const auto & entry : std::filesystem::directory_iterator(reference_directory)) {
    if (entry.path().extension() != ".sonar") {
      continue;
    }
    SCOPED_TRACE(entry.path().string());
    const auto destination = path(std::to_string(files++) + ".sonar");
    std::ifstream stream(entry.path(), std::ios::binary);
    sonar3d::NativeRecorder recorder;
    recorder.start(destination);
    std::vector<std::uint8_t> packet;
    std::size_t packets = 0;
    while (sonar3d::protocol::read_packet(stream, &packet)) {
      ASSERT_TRUE(recorder.enqueue(packet));
      ++packets;
    }
    const auto result = recorder.stop();
    ASSERT_TRUE(result.error.empty()) << result.error;
    EXPECT_EQ(result.packets, packets);
    EXPECT_EQ(read(destination), read(entry.path().string()));
  }
  EXPECT_GT(files, 0U);
}

}  // namespace
