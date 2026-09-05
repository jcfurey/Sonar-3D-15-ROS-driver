// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/sonar_driver.hpp"
#include "stream_test_support.hpp"

namespace
{

using namespace std::chrono_literals;
using Image = sensor_msgs::msg::Image;
using Cloud = sensor_msgs::msg::PointCloud2;
using Imu = sensor_msgs::msg::Imu;
using sonar3d::protocol::BitmapImageType;
using sonar3d::protocol::ProtocolVersion;

class DriverStream : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void start(bool enable_products = true)
  {
    port_ = sonar3d::testing::unused_udp_port();
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(GetParam());
    options.arguments({"--ros-args", "-r", "__ns:=/sonar3d_stream_test"});
    options.append_parameter_override("configure_sonar", false);
    options.append_parameter_override("sonar_ip", "127.0.0.1");
    options.append_parameter_override("multicast_group", "239.255.96.15");
    options.append_parameter_override("multicast_port", port_);
    options.append_parameter_override("multicast_interface", "127.0.0.1");
    options.append_parameter_override("frame_id", "test_sonar");
    options.append_parameter_override("imu_frame_id", "test_imu");
    options.append_parameter_override("diagnostics_period", 0.1);
    for (const auto * parameter : {"publish_range_image", "publish_point_cloud",
        "publish_bitmap_images", "publish_imu"})
    {
      options.append_parameter_override(parameter, enable_products);
    }
    driver_ = std::make_shared<sonar3d::SonarDriver>(options);
    rclcpp::NodeOptions observer_options;
    observer_options.use_intra_process_comms(GetParam());
    observer_ = std::make_shared<rclcpp::Node>(
      "observer", "/sonar3d_stream_test", observer_options);
    diagnostics_ = observer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        if (!message->status.empty() &&
        message->status.front().name == "/sonar3d_stream_test/sonar3d_driver: RIP stream")
        {
          status_ = message->status.front();
        }
      });
    executor_.add_node(driver_);
    executor_.add_node(observer_);
  }

  template<typename Predicate>
  bool wait_for(Predicate ready)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    do {
      executor_.spin_some();
      if (ready()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  std::uint64_t metric(const std::string & key) const
  {
    for (const auto & value : status_.values) {
      if (value.key == key) {
        return std::stoull(value.value);
      }
    }
    return 0;
  }

  void subscribe()
  {
    const auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(100));
    subscriptions_.push_back(observer_->create_subscription<Image>(
        "sonar_range_image", qos, [this](Image::ConstSharedPtr message) {
          ranges_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Cloud>(
        "sonar_point_cloud", qos, [this](Cloud::ConstSharedPtr message) {
          clouds_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Image>(
        "sonar_intensity_image", qos, [this](Image::ConstSharedPtr message) {
          intensities_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Image>(
        "sonar_shaded_image", qos, [this](Image::ConstSharedPtr message) {
          shaded_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Imu>(
        "sonar_imu", qos, [this](Imu::ConstSharedPtr message) {
          imus_.push_back(message);
        }));
  }

  bool discovered()
  {
    return wait_for([this] {
               for (const auto * topic : {"sonar_range_image", "sonar_point_cloud",
                 "sonar_intensity_image", "sonar_shaded_image", "sonar_imu"})
               {
                 if (driver_->count_subscribers(topic) != 1 ||
                 observer_->count_publishers(topic) != 1)
                 {
                   return false;
                 }
               }
               return true;
      });
  }

  void send_frame(std::uint32_t sequence, ProtocolVersion version)
  {
    sender_.send(port_, sonar3d::protocol::encode_packet(
        sonar3d::testing::range_image(sequence, sequence % 2 != 0), version), "239.255.96.15");
    sender_.send(port_, sonar3d::protocol::encode_packet(sonar3d::testing::bitmap_image(
          sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE), version), "239.255.96.15");
    sender_.send(port_, sonar3d::protocol::encode_packet(sonar3d::testing::bitmap_image(
          sequence, BitmapImageType::SHADED_IMAGE), version), "239.255.96.15");
    sender_.send(port_, sonar3d::protocol::encode_packet(sonar3d::testing::imu_batch(sequence)),
      "239.255.96.15");
  }

  void check_frame(std::uint32_t sequence, std::size_t index)
  {
    const auto source = sonar3d::testing::range_image(sequence, sequence % 2 != 0);
    const auto header = sonar3d::conversions::make_header(source.header, "test_sonar",
        builtin_interfaces::msg::Time{});
    EXPECT_EQ(*ranges_.at(index), sonar3d::conversions::make_range_image(source, header));
    EXPECT_EQ(*clouds_.at(index), sonar3d::conversions::make_point_cloud(source, header));
    EXPECT_EQ(*intensities_.at(index), sonar3d::conversions::make_bitmap_image(
        sonar3d::testing::bitmap_image(sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE), header));
    EXPECT_EQ(*shaded_.at(index), sonar3d::conversions::make_bitmap_image(
        sonar3d::testing::bitmap_image(sequence, BitmapImageType::SHADED_IMAGE), header));
    const auto expected_imu = sonar3d::conversions::make_imu_messages(
      sonar3d::testing::imu_batch(sequence), "test_imu", builtin_interfaces::msg::Time{});
    for (std::size_t sample = 0; sample < expected_imu.size(); ++sample) {
      EXPECT_EQ(*imus_.at(index * 5 + sample), expected_imu[sample]);
    }
  }

  sonar3d::testing::UdpSender sender_;
  std::uint16_t port_{};
  std::shared_ptr<sonar3d::SonarDriver> driver_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<Image::ConstSharedPtr> ranges_, intensities_, shaded_;
  std::vector<Cloud::ConstSharedPtr> clouds_;
  std::vector<Imu::ConstSharedPtr> imus_;
  diagnostic_msgs::msg::DiagnosticStatus status_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
};

TEST_P(DriverStream, BothProtocolVersionsDeliverEveryProductFromOneSonar)
{
  start();
  subscribe();
  ASSERT_TRUE(discovered());
  for (std::uint32_t frame = 0; frame < 8; ++frame) {
    send_frame(frame, frame % 2 ? ProtocolVersion::RIP2 : ProtocolVersion::RIP1);
    ASSERT_TRUE(wait_for([this, frame] {
        return ranges_.size() == frame + 1 && clouds_.size() == frame + 1 &&
               intensities_.size() == frame + 1 && shaded_.size() == frame + 1 &&
               imus_.size() == (frame + 1) * 5;
      }));
    check_frame(frame, frame);
  }
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 32;}));
  EXPECT_EQ(metric("range_images"), 8U);
  EXPECT_EQ(metric("bitmap_images"), 16U);
  EXPECT_EQ(metric("imu_batches"), 8U);
  EXPECT_EQ(metric("sequence_gaps"), 0U);
  EXPECT_EQ(metric("malformed_packets"), 0U);
  EXPECT_GT(metric("udp_receive_buffer_bytes"), 0U);
}

TEST_P(DriverStream, SubscribersCanJoinLeaveAndRejoinWhileReceptionContinues)
{
  start();
  send_frame(0, ProtocolVersion::RIP1);
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 4;}));
  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(1, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {
      return ranges_.size() == 1 && clouds_.size() == 1 &&
             intensities_.size() == 1 && shaded_.size() == 1 && imus_.size() == 5;
    }));
  check_frame(1, 0);
  subscriptions_.clear();
  ASSERT_TRUE(wait_for([this] {return driver_->count_subscribers("sonar_point_cloud") == 0;}));
  send_frame(2, ProtocolVersion::RIP1);
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 12;}));
  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(3, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {
      return ranges_.size() == 2 && clouds_.size() == 2 &&
             intensities_.size() == 2 && shaded_.size() == 2 && imus_.size() == 10;
    }));
  check_frame(3, 1);
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 16;}));
  EXPECT_EQ(metric("sequence_gaps"), 0U);
}

TEST_P(DriverStream, DisabledProductsKeepReceivingAndReportingPackets)
{
  start(false);
  subscribe();
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 4;}));
  EXPECT_EQ(observer_->count_publishers("sonar_range_image"), 0U);
  EXPECT_EQ(observer_->count_publishers("sonar_point_cloud"), 0U);
  EXPECT_EQ(observer_->count_publishers("sonar_intensity_image"), 0U);
  EXPECT_EQ(observer_->count_publishers("sonar_shaded_image"), 0U);
  EXPECT_EQ(observer_->count_publishers("sonar_imu"), 0U);
  EXPECT_TRUE(ranges_.empty());
  EXPECT_TRUE(clouds_.empty());
  EXPECT_TRUE(intensities_.empty());
  EXPECT_TRUE(shaded_.empty());
  EXPECT_TRUE(imus_.empty());
}

INSTANTIATE_TEST_SUITE_P(Transport, DriverStream, ::testing::Bool());

}  // namespace
