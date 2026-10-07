// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <zlib.h>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "sonar3d/conversions.hpp"
#include "stream_test_support.hpp"
#include "WaterLinkedSonarIntegrationProtocol.pb.h"

extern char ** environ;

namespace
{

using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;
using Cloud = sensor_msgs::msg::PointCloud2;
using Imu = sensor_msgs::msg::Imu;
using sonar3d::protocol::BitmapImageType;
using PacketBytes = std::vector<std::uint8_t>;

PacketBytes unknown_packet(const std::string & type)
{
  waterlinked::sonar::protocol::Packet envelope;
  envelope.mutable_msg()->set_type_url(type);
  // Only the envelope is understood. Private payloads must not be interpreted
  // as a public ImuBatch, even if their bytes would fail that decoder.
  envelope.mutable_msg()->set_value("\xff");
  const auto payload = envelope.SerializeAsString();
  PacketBytes packet{'R', 'I', 'P', '1'};
  const auto append_u32 = [&packet](std::uint32_t value) {
      for (unsigned shift = 0; shift < 32; shift += 8) {
        packet.push_back(static_cast<std::uint8_t>(value >> shift));
      }
    };
  append_u32(static_cast<std::uint32_t>(payload.size() + 12));
  packet.insert(packet.end(), payload.begin(), payload.end());
  append_u32(crc32(0, packet.data(), packet.size()));
  return packet;
}

class Recording
{
public:
  Recording()
  : Recording(mixed_packets()) {}

  explicit Recording(const std::vector<PacketBytes> & packets)
  {
    char pattern[] = "/tmp/sonar3d-replay-XXXXXX";
    const int descriptor = mkstemp(pattern);
    if (descriptor < 0) {
      throw std::runtime_error("could not create test recording");
    }
    close(descriptor);
    path_ = pattern;
    std::ofstream stream(path_, std::ios::binary);
    for (const auto & packet : packets) {
      write(stream, packet);
    }
    if (!stream) {
      throw std::runtime_error("could not write test recording");
    }
  }

  ~Recording() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  static std::vector<PacketBytes> mixed_packets()
  {
    std::vector<PacketBytes> packets;
    for (std::uint32_t sequence = 0; sequence < 12; ++sequence) {
      const auto version = sequence % 2 ? sonar3d::protocol::ProtocolVersion::RIP2 :
        sonar3d::protocol::ProtocolVersion::RIP1;
      for (const auto & message : std::vector<sonar3d::protocol::Message>{
          sonar3d::testing::range_image(sequence, sequence % 2 != 0),
          sonar3d::testing::bitmap_image(sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE),
          sonar3d::testing::bitmap_image(sequence, BitmapImageType::SHADED_IMAGE)})
      {
        packets.push_back(sonar3d::protocol::encode_packet(message, version));
      }
      packets.push_back(sonar3d::protocol::encode_packet(sonar3d::testing::imu_batch(sequence)));
    }
    return packets;
  }
  static void write(std::ofstream & stream, const std::vector<std::uint8_t> & packet)
  {
    stream.write(reinterpret_cast<const char *>(packet.data()), packet.size());
  }
  std::string path_;
};

class PlayerProcess
{
public:
  explicit PlayerProcess(const std::string & path, bool receive_time)
  : PlayerProcess(path, playback_arguments(receive_time)) {}

  PlayerProcess(const std::string & path, const std::vector<std::string> & options)
  {
    std::vector<std::string> arguments{TEST_SONAR_REPLAY_PATH, "--file", path};
    arguments.insert(arguments.end(), options.begin(), options.end());
    arguments.insert(arguments.end(), {"--ros-args", "-r", "__ns:=/sonar3d_replay_test"});
    std::vector<char *> argv;
    for (auto & argument : arguments) {
      argv.push_back(argument.data());
    }
    argv.push_back(nullptr);
    char pattern[] = "/tmp/sonar3d-replay-output-XXXXXX";
    const int descriptor = mkstemp(pattern);
    if (descriptor < 0) {
      throw std::runtime_error("could not create player output file");
    }
    output_path_ = pattern;
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, descriptor, STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, descriptor, STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, descriptor);
    const auto result = posix_spawn(&pid_, argv.front(), &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(descriptor);
    if (result != 0) {
      std::remove(output_path_.c_str());
      throw std::runtime_error("could not start recording player");
    }
  }

  ~PlayerProcess()
  {
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      waitpid(pid_, &status_, 0);
    }
    std::remove(output_path_.c_str());
  }

  bool finished()
  {
    if (pid_ > 0 && waitpid(pid_, &status_, WNOHANG) == pid_) {
      pid_ = -1;
    }
    return pid_ < 0;
  }

  bool succeeded() const {return WIFEXITED(status_) && WEXITSTATUS(status_) == 0;}

  void interrupt() const
  {
    if (pid_ > 0) {
      kill(pid_, SIGINT);
    }
  }

  std::string output() const
  {
    std::ifstream stream(output_path_);
    return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
  }

  bool wait()
  {
    const auto timeout = std::chrono::steady_clock::now() + 5s;
    while (!finished() && std::chrono::steady_clock::now() < timeout) {
      std::this_thread::sleep_for(2ms);
    }
    return finished();
  }

private:
  static std::vector<std::string> playback_arguments(bool receive_time)
  {
    std::vector<std::string> result{
      "--startup-delay", "1.0", "--realtime-factor", "2.0"};
    if (receive_time) {
      result.push_back("--receive-time");
    }
    return result;
  }

  pid_t pid_{-1};
  int status_{};
  std::string output_path_;
};

class ReplayTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

class RecordingPlayer : public ReplayTest, public ::testing::WithParamInterface<bool> {};

TEST_F(ReplayTest, ValidationChecksAllProductsWithoutSubscribersOrPacing)
{
  const Recording recording;
  PlayerProcess process(recording.path(), std::vector<std::string>{
      "--validate-only", "--startup-delay", "60", "--realtime-factor", "0.001"});
  ASSERT_TRUE(process.wait()) << process.output();
  ASSERT_TRUE(process.succeeded()) << process.output();
  EXPECT_NE(process.output().find(
      "Validated 48 supported packets; unsupported 0; damaged 0; malformed 0; framing errors 0"),
    std::string::npos);
  EXPECT_EQ(process.output().find("Published "), std::string::npos);
}

TEST_F(ReplayTest, ValidationSeparatesUnsupportedCrcFailuresAndMalformedProducts)
{
  const auto orientation = unknown_packet(
    "type.googleapis.com/waterlinked.sonar.internal.ImuOrientation");
  const auto raw = unknown_packet("type.googleapis.com/waterlinked.sonar.internal.ImuRaw");
  auto damaged = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(0));
  damaged.back() ^= 0x01;
  auto range = sonar3d::testing::range_image(0);
  range.pixel_scale = -1;
  auto bitmap = sonar3d::testing::bitmap_image(0, BitmapImageType::SIGNAL_STRENGTH_IMAGE);
  bitmap.pixels.pop_back();
  auto imu = sonar3d::testing::imu_batch(0);
  imu.specific_force.pop_back();
  auto unknown_bitmap = sonar3d::testing::bitmap_image(0, BitmapImageType::SHADED_IMAGE);
  unknown_bitmap.type = static_cast<BitmapImageType>(99);
  const Recording recording({
      orientation, orientation, raw, damaged,
      sonar3d::protocol::encode_packet(range), sonar3d::protocol::encode_packet(bitmap),
      sonar3d::protocol::encode_packet(imu), sonar3d::protocol::encode_packet(unknown_bitmap),
      sonar3d::protocol::encode_packet(sonar3d::testing::range_image(0))});
  PlayerProcess process(recording.path(), std::vector<std::string>{"--validate-only"});
  ASSERT_TRUE(process.wait()) << process.output();
  EXPECT_FALSE(process.succeeded());
  const auto output = process.output();
  EXPECT_NE(output.find(
      "Validated 1 supported packets; unsupported 4; damaged 1; malformed 3; framing errors 0"),
    std::string::npos) << output;
  EXPECT_NE(output.find("ImuOrientation: 2 packets (payload not decoded)"), std::string::npos);
  EXPECT_NE(output.find("ImuRaw: 1 packets (payload not decoded)"), std::string::npos);
  EXPECT_NE(output.find("BitmapImageGreyscale8/type=99: 1 packets"), std::string::npos);
}

TEST_F(ReplayTest, UnsupportedPrivateImuMessagesDoNotFailValidation)
{
  const Recording recording({unknown_packet(
        "type.googleapis.com/waterlinked.sonar.internal.ImuOrientation")});
  PlayerProcess process(recording.path(), std::vector<std::string>{"--validate-only"});
  ASSERT_TRUE(process.wait()) << process.output();
  EXPECT_TRUE(process.succeeded()) << process.output();
  EXPECT_NE(process.output().find(
      "Validated 0 supported packets; unsupported 1; damaged 0; malformed 0; framing errors 0"),
    std::string::npos);
}

TEST_F(ReplayTest, LostFramingFailsWithSummary)
{
  auto packet = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(0));
  packet.pop_back();
  const Recording recording({packet});
  PlayerProcess process(recording.path(), std::vector<std::string>{"--validate-only"});
  ASSERT_TRUE(process.wait()) << process.output();
  EXPECT_FALSE(process.succeeded());
  EXPECT_NE(process.output().find("Lost RIP framing at byte 0"), std::string::npos);
  EXPECT_NE(process.output().find("framing errors 1"), std::string::npos);
}

TEST_F(ReplayTest, PlaybackRecoversAndReportsActualPublicationCounts)
{
  auto damaged = sonar3d::protocol::encode_packet(sonar3d::testing::range_image(0));
  damaged.back() ^= 0x01;
  auto malformed = sonar3d::testing::range_image(0);
  malformed.pixel_scale = -1;
  const Recording recording({
      unknown_packet("type.googleapis.com/waterlinked.sonar.internal.ImuOrientation"),
      damaged, sonar3d::protocol::encode_packet(malformed),
      sonar3d::protocol::encode_packet(sonar3d::testing::range_image(0))});
  auto observer = std::make_shared<rclcpp::Node>("observer", "/sonar3d_replay_test");
  auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(10));
  qos.reliable();
  std::size_t received{};
  const auto subscription = observer->create_subscription<Cloud>(
    "points", qos, [&received](Cloud::ConstSharedPtr) {++received;});
  PlayerProcess process(recording.path(), false);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && (!process.finished() || received == 0)) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(process.finished()) << process.output();
  EXPECT_TRUE(process.succeeded()) << process.output();
  EXPECT_EQ(received, 1U);
  EXPECT_NE(process.output().find(
      "Processed 1 supported packets; unsupported 1; damaged 1; malformed 1; framing errors 0"),
    std::string::npos);
  EXPECT_NE(process.output().find(
      "Published /sonar3d_replay_test/points: 1 ROS messages"), std::string::npos);
}

TEST_F(ReplayTest, InterruptStopsPlaybackCleanlyWithAnIncompleteSummary)
{
  const Recording recording;
  auto observer = std::make_shared<rclcpp::Node>("observer", "/sonar3d_replay_test");
  auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(10));
  qos.reliable();
  std::size_t received{};
  const auto subscription = observer->create_subscription<Cloud>(
    "points", qos, [&received](Cloud::ConstSharedPtr) {++received;});
  PlayerProcess process(recording.path(), std::vector<std::string>{
      "--startup-delay", "1.0", "--realtime-factor", "0.01"});
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(received, 1U) << process.output();
  process.interrupt();
  ASSERT_TRUE(process.wait()) << process.output();
  EXPECT_TRUE(process.succeeded()) << process.output();
  EXPECT_NE(process.output().find("Stopped before recording EOF; results are incomplete"),
    std::string::npos);
  EXPECT_EQ(process.output().find("failed to create guard condition"), std::string::npos);
}

TEST_F(ReplayTest, OutputWritesEveryProductToABagWithoutPacing)
{
  const Recording recording;
  char pattern[] = "/tmp/sonar3d-bag-XXXXXX";
  ASSERT_NE(mkdtemp(pattern), nullptr);
  const std::filesystem::path directory(pattern);
  const auto bag = (directory / "converted").string();
  {
    // Neither the startup delay nor the 1000x slower pacing may apply.
    PlayerProcess process(recording.path(), std::vector<std::string>{
        "--output", bag, "--startup-delay", "60", "--realtime-factor", "0.001"});
    ASSERT_TRUE(process.wait()) << process.output();
    ASSERT_TRUE(process.succeeded()) << process.output();
    EXPECT_NE(process.output().find(
        "Wrote /sonar3d_replay_test/points: 12 ROS messages"), std::string::npos)
      << process.output();
  }

  // Messages borrow memory from the storage plugin, so they must be released
  // before the reader unloads it.
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  std::map<std::string, std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>>
  topics;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    topics[message->topic_name].push_back(message);
  }
  std::filesystem::remove_all(directory);

  ASSERT_EQ(topics["/sonar3d_replay_test/range_image"].size(), 12U);
  ASSERT_EQ(topics["/sonar3d_replay_test/points"].size(), 12U);
  ASSERT_EQ(topics["/sonar3d_replay_test/intensity_image"].size(), 12U);
  ASSERT_EQ(topics["/sonar3d_replay_test/shaded_image"].size(), 12U);
  ASSERT_EQ(topics["/sonar3d_replay_test/imu/data_raw"].size(), 60U);
  ASSERT_EQ(topics["/sonar3d_replay_test/image_metadata"].size(), 36U);

  rclcpp::Serialization<sonar3d::msg::ImageMetadata> metadata_serialization;
  for (const auto & stored : topics["/sonar3d_replay_test/image_metadata"]) {
    sonar3d::msg::ImageMetadata metadata;
    const rclcpp::SerializedMessage serialized(*stored->serialized_data);
    metadata_serialization.deserialize_message(&serialized, &metadata);
    EXPECT_EQ(stored->recv_timestamp, rclcpp::Time(metadata.header.stamp).nanoseconds());
    EXPECT_TRUE(metadata.sensor_timestamp_valid);
    EXPECT_EQ(metadata.width, 256U);
    EXPECT_EQ(metadata.height, 64U);
    EXPECT_EQ(metadata.driver_source_sha256.size(), 64U);
  }

  rclcpp::Serialization<Cloud> cloud_serialization;
  rclcpp::Serialization<Imu> imu_serialization;
  for (std::uint32_t sequence = 0; sequence < 12; ++sequence) {
    const auto source = sonar3d::testing::range_image(sequence, sequence % 2 != 0);
    const auto header = sonar3d::conversions::make_header(
      source.header, "sonar3d_link", builtin_interfaces::msg::Time{});
    const auto & stored = topics["/sonar3d_replay_test/points"][sequence];
    Cloud cloud;
    const rclcpp::SerializedMessage serialized(*stored->serialized_data);
    cloud_serialization.deserialize_message(&serialized, &cloud);
    const auto signal = sonar3d::testing::bitmap_image(
      sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE);
    EXPECT_EQ(cloud, sonar3d::conversions::make_point_cloud(source, header, true, &signal));
    EXPECT_EQ(stored->recv_timestamp, rclcpp::Time(header.stamp).nanoseconds());
  }
  const auto expected_imu = sonar3d::conversions::make_imu_messages(
    sonar3d::testing::imu_batch(3), "sonar3d_imu_link", builtin_interfaces::msg::Time{});
  for (std::size_t sample = 0; sample < expected_imu.size(); ++sample) {
    Imu imu;
    const rclcpp::SerializedMessage serialized(
      *topics["/sonar3d_replay_test/imu/data_raw"][15 + sample]->serialized_data);
    imu_serialization.deserialize_message(&serialized, &imu);
    EXPECT_EQ(imu, expected_imu[sample]);
  }
}

TEST_F(ReplayTest, OutputRejectsReceiveTimeStamps)
{
  const Recording recording;
  PlayerProcess process(recording.path(), std::vector<std::string>{
      "--output", "/tmp/sonar3d-unused-bag", "--receive-time"});
  ASSERT_TRUE(process.wait()) << process.output();
  EXPECT_FALSE(process.succeeded());
  EXPECT_NE(process.output().find("--receive-time cannot be used with --output"),
    std::string::npos) << process.output();
}

TEST_P(RecordingPlayer, MixedRecordingDeliversAllProductsWithSensorOrReceiveTimes)
{
  const Recording recording;
  auto observer = std::make_shared<rclcpp::Node>("observer", "/sonar3d_replay_test");
  auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(100));
  qos.reliable();
  std::vector<Image::ConstSharedPtr> ranges, intensities, shaded;
  std::vector<Cloud::ConstSharedPtr> clouds;
  std::vector<Imu::ConstSharedPtr> imus;
  const auto range_sub = observer->create_subscription<Image>(
    "range_image", qos, [&ranges](Image::ConstSharedPtr message) {
      ranges.push_back(message);
      });
  const auto cloud_sub = observer->create_subscription<Cloud>(
    "points", qos, [&clouds](Cloud::ConstSharedPtr message) {
      clouds.push_back(message);
      });
  const auto intensity_sub = observer->create_subscription<Image>(
    "intensity_image", qos, [&intensities](Image::ConstSharedPtr message) {
      intensities.push_back(message);
    });
  const auto shaded_sub = observer->create_subscription<Image>(
    "shaded_image", qos, [&shaded](Image::ConstSharedPtr message) {
      shaded.push_back(message);
      });
  const auto imu_sub = observer->create_subscription<Imu>(
    "imu/data_raw", qos, [&imus](Imu::ConstSharedPtr message) {imus.push_back(message);});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  const builtin_interfaces::msg::Time started = observer->now();
  PlayerProcess process(recording.path(), GetParam());
  const auto timeout = std::chrono::steady_clock::now() + 10s;
  while (std::chrono::steady_clock::now() < timeout) {
    executor.spin_some();
    if (process.finished() && ranges.size() == 12 && clouds.size() == 12 &&
      intensities.size() == 12 && shaded.size() == 12 && imus.size() == 60)
    {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_TRUE(process.finished());
  ASSERT_TRUE(process.succeeded());
  ASSERT_EQ(ranges.size(), 12U);
  ASSERT_EQ(clouds.size(), 12U);
  ASSERT_EQ(intensities.size(), 12U);
  ASSERT_EQ(shaded.size(), 12U);
  ASSERT_EQ(imus.size(), 60U);
  EXPECT_NE(process.output().find("Processed 48 supported packets; unsupported 0; damaged 0"),
    std::string::npos);
  EXPECT_NE(process.output().find(
      "Published /sonar3d_replay_test/points: 12 ROS messages"),
    std::string::npos);
  EXPECT_NE(process.output().find("Published /sonar3d_replay_test/imu/data_raw: 60 ROS messages"),
    std::string::npos);
  for (std::uint32_t sequence = 0; sequence < 12; ++sequence) {
    const auto source = sonar3d::testing::range_image(sequence, sequence % 2 != 0);
    const auto header = sonar3d::conversions::make_header(source.header, "sonar3d_link",
        builtin_interfaces::msg::Time{});
    EXPECT_EQ(ranges[sequence]->data, sonar3d::conversions::make_range_image(source, header).data);
    const auto signal = sonar3d::testing::bitmap_image(
      sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE);
    EXPECT_EQ(
      clouds[sequence]->data,
      sonar3d::conversions::make_point_cloud(source, header, true, &signal).data);
    EXPECT_EQ(intensities[sequence]->data, sonar3d::conversions::make_bitmap_image(
        sonar3d::testing::bitmap_image(sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE),
        header).data);
    EXPECT_EQ(shaded[sequence]->data, sonar3d::conversions::make_bitmap_image(
        sonar3d::testing::bitmap_image(sequence, BitmapImageType::SHADED_IMAGE), header).data);
    for (const auto & actual : {ranges[sequence]->header, clouds[sequence]->header,
        intensities[sequence]->header, shaded[sequence]->header})
    {
      EXPECT_EQ(actual.frame_id, "sonar3d_link");
      if (GetParam()) {
        EXPECT_GE(rclcpp::Time(actual.stamp).nanoseconds(), rclcpp::Time(started).nanoseconds());
      } else {
        EXPECT_EQ(actual.stamp, header.stamp);
      }
    }
    const auto expected = sonar3d::conversions::make_imu_messages(
      sonar3d::testing::imu_batch(sequence), "sonar3d_imu_link", builtin_interfaces::msg::Time{});
    for (std::size_t sample = 0; sample < expected.size(); ++sample) {
      const auto & actual = *imus[sequence * 5 + sample];
      EXPECT_EQ(actual.linear_acceleration, expected[sample].linear_acceleration);
      EXPECT_EQ(actual.angular_velocity, expected[sample].angular_velocity);
      EXPECT_EQ(actual.header.frame_id, "sonar3d_imu_link");
      if (GetParam()) {
        EXPECT_GE(rclcpp::Time(actual.header.stamp).nanoseconds(),
          rclcpp::Time(started).nanoseconds());
      } else {
        EXPECT_EQ(actual.header.stamp, expected[sample].header.stamp);
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Timestamps, RecordingPlayer, ::testing::Bool());

}  // namespace
