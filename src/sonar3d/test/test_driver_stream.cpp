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

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "http_test_server.hpp"
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

  // The fixtures carry fixed 2023 sensor timestamps, so exact-stamp tests turn
  // the sensor clock check off; the clock tests enable it explicitly.
  void start(
    bool enable_products = true, double max_sensor_clock_offset = 0.0,
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    port_ = sonar3d::testing::unused_udp_port();
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(GetParam());
    options.arguments({"--ros-args", "-r", "__ns:=/sonar3d_stream_test"});
    options.append_parameter_override("configure_sonar", false);
    options.append_parameter_override("sonar_ip", "127.0.0.1");
    options.append_parameter_override("multicast_group", "239.255.96.15");
    options.append_parameter_override("udp_port", port_);
    options.append_parameter_override("interface_address", "127.0.0.1");
    options.append_parameter_override("device_status_period", 0.0);
    options.append_parameter_override("frame_id", "test_sonar");
    options.append_parameter_override("imu_frame_id", "test_imu");
    options.append_parameter_override("diagnostic_updater.period", 0.1);
    options.append_parameter_override("max_sensor_clock_offset", max_sensor_clock_offset);
    for (const auto * parameter : {"publish_range_image", "publish_point_cloud",
        "publish_bitmap_images", "publish_imu"})
    {
      options.append_parameter_override(parameter, enable_products);
    }
    for (const auto & parameter : overrides) {
      options.parameter_overrides().push_back(parameter);
    }
    driver_ = std::make_shared<sonar3d::SonarDriver>(options);
    rclcpp::NodeOptions observer_options;
    observer_options.use_intra_process_comms(GetParam());
    observer_ = std::make_shared<rclcpp::Node>(
      "observer", "/sonar3d_stream_test", observer_options);
    diagnostics_ = observer_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        for (const auto & status : message->status) {
          if (status.name.ends_with("sonar3d_driver: RIP stream")) {
            status_ = status;
          } else if (status.name.ends_with("sonar3d_driver: Range image rate")) {
            rate_status_ = status;
          } else if (status.name.ends_with("sonar3d_driver: Device")) {
            device_status_ = status;
          }
        }
      });
    executor_.add_node(driver_->get_node_base_interface());
    executor_.add_node(observer_);
    ASSERT_EQ(driver_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(driver_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
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

  std::string value(const std::string & key) const
  {
    for (const auto & item : status_.values) {
      if (item.key == key) {
        return item.value;
      }
    }
    return {};
  }

  std::string device_value(const std::string & key) const
  {
    for (const auto & item : device_status_.values) {
      if (item.key == key) {
        return item.value;
      }
    }
    return {};
  }

  std::uint64_t metric(const std::string & key) const
  {
    const auto text = value(key);
    return text.empty() ? 0 : std::stoull(text);
  }

  void subscribe()
  {
    const auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(100));
    subscriptions_.push_back(observer_->create_subscription<Image>(
        "range_image", qos, [this](Image::ConstSharedPtr message) {
          ranges_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Cloud>(
        "points", qos, [this](Cloud::ConstSharedPtr message) {
          clouds_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Image>(
        "intensity_image", qos, [this](Image::ConstSharedPtr message) {
          intensities_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Image>(
        "shaded_image", qos, [this](Image::ConstSharedPtr message) {
          shaded_.push_back(message);
        }));
    subscriptions_.push_back(observer_->create_subscription<Imu>(
        "imu/data_raw", qos, [this](Imu::ConstSharedPtr message) {
          imus_.push_back(message);
        }));
  }

  bool discovered()
  {
    return wait_for([this] {
               for (const auto * topic : {"range_image", "points",
                 "intensity_image", "shaded_image", "imu/data_raw"})
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
        sonar3d::testing::range_image(sequence, sequence % 2 != 0), version), destination_.c_str());
    sender_.send(port_, sonar3d::protocol::encode_packet(sonar3d::testing::bitmap_image(
          sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE), version), destination_.c_str());
    sender_.send(port_, sonar3d::protocol::encode_packet(sonar3d::testing::bitmap_image(
          sequence, BitmapImageType::SHADED_IMAGE), version), destination_.c_str());
    sender_.send(port_, sonar3d::protocol::encode_packet(sonar3d::testing::imu_batch(sequence)),
      destination_.c_str());
  }

  void check_frame(std::uint32_t sequence, std::size_t index)
  {
    const auto source = sonar3d::testing::range_image(sequence, sequence % 2 != 0);
    const auto header = sonar3d::conversions::make_header(source.header, "test_sonar",
        builtin_interfaces::msg::Time{});
    EXPECT_EQ(*ranges_.at(index), sonar3d::conversions::make_range_image(source, header));
    const auto signal = sonar3d::testing::bitmap_image(
      sequence, BitmapImageType::SIGNAL_STRENGTH_IMAGE);
    EXPECT_EQ(
      *clouds_.at(index), sonar3d::conversions::make_point_cloud(source, header, true, &signal));
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
  diagnostic_msgs::msg::DiagnosticStatus rate_status_;
  diagnostic_msgs::msg::DiagnosticStatus device_status_;
  std::string destination_{"239.255.96.15"};
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
  ASSERT_TRUE(wait_for([this] {return driver_->count_subscribers("points") == 0;}));
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
  EXPECT_EQ(observer_->count_publishers("range_image"), 0U);
  EXPECT_EQ(observer_->count_publishers("points"), 0U);
  EXPECT_EQ(observer_->count_publishers("intensity_image"), 0U);
  EXPECT_EQ(observer_->count_publishers("shaded_image"), 0U);
  EXPECT_EQ(observer_->count_publishers("imu/data_raw"), 0U);
  EXPECT_TRUE(ranges_.empty());
  EXPECT_TRUE(clouds_.empty());
  EXPECT_TRUE(intensities_.empty());
  EXPECT_TRUE(shaded_.empty());
  EXPECT_TRUE(imus_.empty());
}

TEST_P(DriverStream, UnsynchronizedSonarClockIsReanchoredToReceiveTime)
{
  start(true, 1.0);
  subscribe();
  ASSERT_TRUE(discovered());
  const rclcpp::Time before = observer_->now();
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {
      return ranges_.size() == 1 && clouds_.size() == 1 && intensities_.size() == 1 &&
             shaded_.size() == 1 && imus_.size() == 5;
    }));
  const rclcpp::Time after = observer_->now();

  const auto received = [&before, &after](const builtin_interfaces::msg::Time & stamp) {
      const rclcpp::Time time(stamp);
      return time >= before && time <= after;
    };
  EXPECT_TRUE(received(ranges_[0]->header.stamp));
  EXPECT_EQ(clouds_[0]->header.stamp, ranges_[0]->header.stamp);
  EXPECT_TRUE(received(intensities_[0]->header.stamp));
  EXPECT_TRUE(received(shaded_[0]->header.stamp));
  EXPECT_TRUE(received(imus_[4]->header.stamp));
  for (std::size_t sample = 1; sample < 5; ++sample) {
    EXPECT_EQ(
      (rclcpp::Time(imus_[sample]->header.stamp) -
      rclcpp::Time(imus_[sample - 1]->header.stamp)).nanoseconds(),
      10'000'000);
  }

  ASSERT_TRUE(wait_for([this] {return metric("sensor_clock_fallbacks") == 4;}));
  EXPECT_EQ(value("timestamp_source"), "receive-aligned (sensor clock unsynchronized)");
  EXPECT_EQ(status_.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_GT(std::stod(value("sensor_clock_offset_seconds")), 1.0e7);
}

TEST_P(DriverStream, SynchronizedSonarClockKeepsSensorTimestamps)
{
  start(true, 1.0);
  subscribe();
  ASSERT_TRUE(discovered());
  auto image = sonar3d::testing::range_image(0);
  image.header.timestamp = sonar3d::testing::timestamp(
    rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() - 20'000'000);
  sender_.send(port_, sonar3d::protocol::encode_packet(image), "239.255.96.15");
  ASSERT_TRUE(wait_for([this] {return ranges_.size() == 1 && clouds_.size() == 1;}));

  builtin_interfaces::msg::Time expected;
  expected.sec = static_cast<std::int32_t>(image.header.timestamp->seconds);
  expected.nanosec = static_cast<std::uint32_t>(image.header.timestamp->nanoseconds);
  EXPECT_EQ(ranges_[0]->header.stamp, expected);
  EXPECT_EQ(clouds_[0]->header.stamp, expected);

  ASSERT_TRUE(wait_for([this] {return value("timestamp_source") == "sensor";}));
  EXPECT_EQ(metric("sensor_clock_fallbacks"), 0U);
  EXPECT_EQ(status_.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  const auto offset = std::stod(value("sensor_clock_offset_seconds"));
  EXPECT_GT(offset, 0.0);
  EXPECT_LT(offset, 1.0);
}

TEST_P(DriverStream, FallbackSourceIsAcceptedAlongsideTheConfiguredSonar)
{
  // The loopback sender stands in for the sonar's fixed fallback address.
  start(true, 0.0, {
      rclcpp::Parameter("sonar_ip", "10.255.255.1"),
      rclcpp::Parameter("fallback_ip", "127.0.0.1")});
  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {
      return ranges_.size() == 1 && clouds_.size() == 1 && imus_.size() == 5;
    }));
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 4;}));
  EXPECT_EQ(metric("rejected_sources"), 0U);
}

TEST_P(DriverStream, OtherSourcesAreRejectedWithoutAFallback)
{
  start(true, 0.0, {
      rclcpp::Parameter("sonar_ip", "10.255.255.1"),
      rclcpp::Parameter("fallback_ip", "")});
  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {return metric("rejected_sources") == 4;}));
  EXPECT_EQ(metric("valid_packets"), 0U);
  EXPECT_TRUE(ranges_.empty());
}

TEST_P(DriverStream, UnsynchronizedClockKeepsSensorSpacingAcrossProducts)
{
  start(true, 1.0);
  subscribe();
  ASSERT_TRUE(discovered());
  const auto batch = sonar3d::testing::imu_batch(0);
  sender_.send(port_, sonar3d::protocol::encode_packet(batch), "239.255.96.15");
  ASSERT_TRUE(wait_for([this] {return imus_.size() == 5;}));

  // Deliver the image well after its acquisition. Receive-time stamping
  // would add that delay; the shared offset keeps the sonar's 30 ms spacing.
  std::this_thread::sleep_for(100ms);
  auto image = sonar3d::testing::range_image(0);
  image.header.timestamp = sonar3d::testing::timestamp(
    *sonar3d::conversions::sensor_nanoseconds(batch.timestamps.back()) + 30'000'000);
  sender_.send(port_, sonar3d::protocol::encode_packet(image), "239.255.96.15");
  ASSERT_TRUE(wait_for([this] {return ranges_.size() == 1 && clouds_.size() == 1;}));

  EXPECT_EQ(
    (rclcpp::Time(ranges_[0]->header.stamp) - rclcpp::Time(imus_[4]->header.stamp)).nanoseconds(),
    30'000'000);
  EXPECT_EQ(clouds_[0]->header.stamp, ranges_[0]->header.stamp);
}

TEST_P(DriverStream, ReliableSubscribersReceiveProductsAsRep2003Requires)
{
  start();
  std::vector<Cloud::ConstSharedPtr> reliable;
  const auto subscription = observer_->create_subscription<Cloud>(
    "points", rclcpp::QoS(10).reliable(), [&reliable](Cloud::ConstSharedPtr message) {
      reliable.push_back(message);
    });
  ASSERT_TRUE(wait_for([this] {return driver_->count_subscribers("points") == 1;}));
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([&reliable] {return reliable.size() == 1;}));
}

TEST_P(DriverStream, DeactivationPausesStreamingAndDropsStaleData)
{
  start();
  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {return ranges_.size() == 1 && imus_.size() == 5;}));
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 4;}));

  ASSERT_EQ(driver_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(wait_for([this] {return status_.message.starts_with("not streaming");}));
  EXPECT_EQ(value("lifecycle_state"), "inactive");
  send_frame(5, ProtocolVersion::RIP2);
  std::this_thread::sleep_for(100ms);
  executor_.spin_some();
  EXPECT_EQ(ranges_.size(), 1U);

  // The frame sent while inactive is discarded; the next one is published and
  // the sequence jump across the pause is not counted as lost frames.
  ASSERT_EQ(driver_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  send_frame(9, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {return ranges_.size() == 2 && imus_.size() == 10;}));
  check_frame(9, 1);
  ASSERT_TRUE(wait_for([this] {return metric("valid_packets") == 8;}));
  EXPECT_EQ(metric("sequence_gaps"), 0U);
}

TEST_P(DriverStream, ParametersChangeOnlyWhileUnconfigured)
{
  start();
  EXPECT_FALSE(driver_->set_parameter(rclcpp::Parameter("frame_id", "moved")).successful);
  // Parameters owned by other components stay adjustable.
  EXPECT_TRUE(driver_->set_parameter(rclcpp::Parameter("diagnostic_updater.period", 0.1))
    .successful);

  ASSERT_EQ(driver_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(driver_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_TRUE(driver_->set_parameter(rclcpp::Parameter("frame_id", "moved")).successful);
  ASSERT_EQ(driver_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(driver_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {return ranges_.size() == 1 && clouds_.size() == 1;}));
  EXPECT_EQ(ranges_[0]->header.frame_id, "moved");
  EXPECT_EQ(clouds_[0]->header.frame_id, "moved");
}

TEST_P(DriverStream, InvalidParametersFailConfigurationAndCanBeCorrected)
{
  start();
  ASSERT_EQ(driver_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(driver_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_TRUE(driver_->set_parameter(rclcpp::Parameter("sonar_ip", "sonar.local")).successful);
  EXPECT_EQ(driver_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_TRUE(driver_->set_parameter(rclcpp::Parameter("sonar_ip", "127.0.0.1")).successful);
  EXPECT_EQ(driver_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_P(DriverStream, RangeImageRateIsDiagnosedWhileActive)
{
  start();
  for (std::uint32_t frame = 0; frame < 6; ++frame) {
    send_frame(frame, ProtocolVersion::RIP2);
    std::this_thread::sleep_for(50ms);
    executor_.spin_some();
  }
  ASSERT_TRUE(wait_for([this] {return !rate_status_.name.empty();}));
  EXPECT_EQ(rate_status_.hardware_id, "127.0.0.1");
}

TEST_P(DriverStream, AutostartParameterActivatesOnceSpinning)
{
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(GetParam());
  options.arguments({"--ros-args", "-r", "__ns:=/sonar3d_autostart_test"});
  options.append_parameter_override("autostart", true);
  options.append_parameter_override("configure_sonar", false);
  options.append_parameter_override("multicast_group", "239.255.96.15");
  options.append_parameter_override("udp_port", sonar3d::testing::unused_udp_port());
  options.append_parameter_override("interface_address", "127.0.0.1");
  const auto driver = std::make_shared<sonar3d::SonarDriver>(options);
  EXPECT_EQ(
    driver->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver->get_node_base_interface());
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (driver->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
    std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(driver->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_P(DriverStream, UnicastModeReceivesDatagramsAddressedToTheDriver)
{
  start(true, 0.0, {rclcpp::Parameter("udp_mode", "unicast")});
  destination_ = "127.0.0.1";
  subscribe();
  ASSERT_TRUE(discovered());
  send_frame(0, ProtocolVersion::RIP2);
  ASSERT_TRUE(wait_for([this] {
      return ranges_.size() == 1 && clouds_.size() == 1 && intensities_.size() == 1 &&
             imus_.size() == 5;
    }));
  check_frame(0, 0);
}

TEST_P(DriverStream, RangeImageWithoutSignalImageIsPublishedAfterAShortWait)
{
  start();
  subscribe();
  ASSERT_TRUE(discovered());
  const auto source = sonar3d::testing::range_image(4);
  const auto sent = std::chrono::steady_clock::now();
  sender_.send(port_, sonar3d::protocol::encode_packet(source), destination_.c_str());
  ASSERT_TRUE(wait_for([this] {return clouds_.size() == 1;}));
  EXPECT_LT(std::chrono::steady_clock::now() - sent, 1s);
  const auto header = sonar3d::conversions::make_header(
    source.header, "test_sonar", builtin_interfaces::msg::Time{});
  EXPECT_EQ(*clouds_[0], sonar3d::conversions::make_point_cloud(source, header, true));
  ASSERT_TRUE(wait_for([this] {return metric("unpaired_range_images") == 1;}));
}

TEST_P(DriverStream, IntensityCanBeLeftOutOfThePointCloud)
{
  start(true, 0.0, {rclcpp::Parameter("point_cloud_intensity", false)});
  subscribe();
  ASSERT_TRUE(discovered());
  const auto source = sonar3d::testing::range_image(2);
  sender_.send(port_, sonar3d::protocol::encode_packet(source), destination_.c_str());
  ASSERT_TRUE(wait_for([this] {return clouds_.size() == 1;}));
  const auto header = sonar3d::conversions::make_header(
    source.header, "test_sonar", builtin_interfaces::msg::Time{});
  EXPECT_EQ(*clouds_[0], sonar3d::conversions::make_point_cloud(source, header));
  EXPECT_EQ(clouds_[0]->fields.size(), 6U);
}

TEST_P(DriverStream, ConfiguresTheSonarAndReportsDeviceStatus)
{
  sonar3d::testing::HttpServer sonar;
  const std::string api = "/api/v1/integration";
  for (const auto * path : {"/time/ntp", "/acoustics/salinity", "/acoustics/speed_of_sound",
      "/acoustics/mode", "/acoustics/range", "/acoustics/enabled", "/udp"})
  {
    sonar.respond("POST " + api + path, {});
  }
  // Firmware before 1.8.0: no public ImuBatch output.
  sonar.respond("GET " + api + "/about", {200,
      R"json({"chipid":"0xC0FFEE","hardware_revision":6,"is_ready":true,"product_id":21045,)json"
      R"json("product_name":"Sonar 3D-15","variant":"","version":"1.7.1 (test)",)json"
      R"json("version_short":"1.7.1"})json", 0ms});
  sonar.respond("GET " + api + "/status", {200,
      R"({"api":{"id":"api","message":"API ok","operational":true,"status":"ok"},)"
      R"("temperature":{"id":"temp","message":"Sonar is warm","operational":true,)"
      R"("status":"warning"},"systems_check":{"id":"sys","message":"OK","operational":true,)"
      R"("status":"ok"},"time":{"id":"time","message":"Not synchronized","operational":true,)"
      R"("status":"ok"}})", 0ms});
  sonar.respond("GET " + api + "/temperature", {200, "41.5", 0ms});
  sonar.respond("GET " + api + "/time/status", {200,
      R"({"system_time":"2026-10-06T19:00:00Z","ntp_synced":false,"ntp_synced_to":"",)"
      R"("ntp_seconds_since_last_sync":null})", 0ms});

  start(true, 0.0, {
      rclcpp::Parameter("configure_sonar", true),
      rclcpp::Parameter("http_port", static_cast<int>(sonar.port())),
      rclcpp::Parameter("speed_of_sound", 0.0),
      rclcpp::Parameter("salinity", "salt"),
      rclcpp::Parameter("acoustics_mode", "high-frequency"),
      rclcpp::Parameter("range_min", 0.5),
      rclcpp::Parameter("range_max", 12.0),
      rclcpp::Parameter("ntp_server", "auto"),
      rclcpp::Parameter("device_status_period", 0.1)});

  ASSERT_TRUE(wait_for([this] {
      return value("configuration") == "successful" && device_value("chip_id") == "0xC0FFEE";
    }));
  EXPECT_EQ(value("configuration_unsupported"), "imu_output");
  EXPECT_EQ(device_status_.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(device_status_.message, "Sonar is warm");
  EXPECT_EQ(device_value("firmware_version"), "1.7.1 (test)");
  EXPECT_EQ(device_value("temperature_celsius"), "41.5");
  EXPECT_EQ(device_value("ntp_synced"), "false");
  ASSERT_TRUE(wait_for([this] {return device_status_.hardware_id == "Sonar 3D-15 0xC0FFEE";}));

  std::map<std::string, std::string> posted;
  for (const auto & request : sonar.requests()) {
    if (request.method == "POST") {
      posted[request.path.substr(api.size())] = request.body;
    }
  }
  EXPECT_EQ(posted["/acoustics/salinity"], R"("salt")");
  EXPECT_EQ(posted["/acoustics/speed_of_sound"], "0.0");
  EXPECT_EQ(posted["/acoustics/mode"], R"("high-frequency")");
  EXPECT_EQ(posted["/time/ntp"], R"({"ntp_address":"auto"})");
  EXPECT_EQ(posted["/output/imu-batch/enabled"], "true");
}

INSTANTIATE_TEST_SUITE_P(Transport, DriverStream, ::testing::Bool());

}  // namespace
