// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sonar3d/conversions.hpp"
#include "stream_test_support.hpp"

extern char ** environ;

namespace
{

using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;
using Cloud = sensor_msgs::msg::PointCloud2;
using Imu = sensor_msgs::msg::Imu;
using sonar3d::protocol::BitmapImageType;

class Recording
{
public:
  Recording()
  {
    char pattern[] = "/tmp/sonar3d-replay-XXXXXX";
    const int descriptor = mkstemp(pattern);
    if (descriptor < 0) {
      throw std::runtime_error("could not create test recording");
    }
    close(descriptor);
    path_ = pattern;
    std::ofstream stream(path_, std::ios::binary);
    for (std::uint32_t sequence = 0; sequence < 12; ++sequence) {
      const auto version = sequence % 2 ? sonar3d::protocol::ProtocolVersion::RIP2 :
        sonar3d::protocol::ProtocolVersion::RIP1;
      for (const auto & message : std::vector<sonar3d::protocol::Message>{
          sonar3d::testing::range_image(sequence, sequence % 2 != 0),
          sonar3d::testing::bitmap_image(sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE),
          sonar3d::testing::bitmap_image(sequence, BitmapImageType::SHADED_IMAGE)})
      {
        write(stream, sonar3d::protocol::encode_packet(message, version));
      }
      write(stream, sonar3d::protocol::encode_packet(sonar3d::testing::imu_batch(sequence)));
    }
    if (!stream) {
      throw std::runtime_error("could not write test recording");
    }
  }

  ~Recording() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
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
  {
    std::vector<std::string> arguments{
      TEST_SONAR_REPLAY_PATH, "--file", path, "--startup-delay", "1.0",
      "--realtime-factor", "2.0"};
    if (receive_time) {
      arguments.push_back("--receive-time");
    }
    arguments.insert(arguments.end(), {"--ros-args", "-r", "__ns:=/sonar3d_replay_test"});
    std::vector<char *> argv;
    for (auto & argument : arguments) {
      argv.push_back(argument.data());
    }
    argv.push_back(nullptr);
    if (posix_spawn(&pid_, argv.front(), nullptr, nullptr, argv.data(), environ) != 0) {
      throw std::runtime_error("could not start recording player");
    }
  }

  ~PlayerProcess()
  {
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
      waitpid(pid_, &status_, 0);
    }
  }

  bool finished()
  {
    if (pid_ > 0 && waitpid(pid_, &status_, WNOHANG) == pid_) {
      pid_ = -1;
    }
    return pid_ < 0;
  }

  bool succeeded() const {return WIFEXITED(status_) && WEXITSTATUS(status_) == 0;}

private:
  pid_t pid_{-1};
  int status_{};
};

class RecordingPlayer : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

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
    "sonar_range_image", qos, [&ranges](Image::ConstSharedPtr message) {
      ranges.push_back(message);
      });
  const auto cloud_sub = observer->create_subscription<Cloud>(
    "sonar_point_cloud", qos, [&clouds](Cloud::ConstSharedPtr message) {
      clouds.push_back(message);
      });
  const auto intensity_sub = observer->create_subscription<Image>(
    "sonar_intensity_image", qos, [&intensities](Image::ConstSharedPtr message) {
      intensities.push_back(message);
    });
  const auto shaded_sub = observer->create_subscription<Image>(
    "sonar_shaded_image", qos, [&shaded](Image::ConstSharedPtr message) {
      shaded.push_back(message);
      });
  const auto imu_sub = observer->create_subscription<Imu>(
    "sonar_imu", qos, [&imus](Imu::ConstSharedPtr message) {imus.push_back(message);});
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
  for (std::uint32_t sequence = 0; sequence < 12; ++sequence) {
    const auto source = sonar3d::testing::range_image(sequence, sequence % 2 != 0);
    const auto header = sonar3d::conversions::make_header(source.header, "sonar3d_link",
        builtin_interfaces::msg::Time{});
    EXPECT_EQ(ranges[sequence]->data, sonar3d::conversions::make_range_image(source, header).data);
    EXPECT_EQ(clouds[sequence]->data, sonar3d::conversions::make_point_cloud(source, header).data);
    EXPECT_EQ(intensities[sequence]->data, sonar3d::testing::bitmap_image(
        sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE).pixels);
    EXPECT_EQ(shaded[sequence]->data, sonar3d::testing::bitmap_image(
        sequence, BitmapImageType::SHADED_IMAGE).pixels);
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
