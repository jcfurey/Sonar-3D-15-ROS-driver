// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/http_client.hpp"
#include "sonar3d/protocol.hpp"
#include "sonar3d/range_signal_pairing.hpp"
#include "sonar3d/sensor_clock.hpp"
#include "sonar3d/sonar3d_parameters.hpp"
#include "sonar3d/udp_receiver.hpp"

namespace sonar3d
{

// Managed (lifecycle) Sonar 3D-15 driver.
//   configure:  validate parameters, open the UDP socket, create publishers,
//               broadcast the IMU transform, start HTTP setup and device
//               status polling.
//   activate:   discard queued datagrams and start streaming.
//   deactivate: stop streaming; the socket and device setup are kept.
//   cleanup:    release the socket, publishers, HTTP setup and polling.
// /diagnostics is published in every state.
class SonarDriver final : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SonarDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SonarDriver() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  template<typename MessageT>
  using Publisher = typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr;

  struct SequenceState
  {
    std::optional<std::uint32_t> last;
  };

  struct Metrics
  {
    std::atomic<std::uint64_t> datagrams{0};
    std::atomic<std::uint64_t> rejected_sources{0};
    std::atomic<std::uint64_t> valid_packets{0};
    std::atomic<std::uint64_t> malformed_packets{0};
    std::atomic<std::uint64_t> crc_failures{0};
    std::atomic<std::uint64_t> unknown_messages{0};
    std::atomic<std::uint64_t> range_images{0};
    std::atomic<std::uint64_t> bitmap_images{0};
    std::atomic<std::uint64_t> imu_batches{0};
    std::atomic<std::uint64_t> sequence_gaps{0};
    std::atomic<std::uint64_t> duplicate_messages{0};
    std::atomic<std::uint64_t> out_of_order_messages{0};
    std::atomic<std::uint64_t> sequence_resets{0};
    std::atomic<std::uint64_t> socket_errors{0};
    std::atomic<std::uint64_t> sensor_clock_fallbacks{0};
    std::atomic<std::uint64_t> unpaired_range_images{0};
  };

  // Which clock stamps the products: no sensor timestamp seen yet, the sonar's
  // own clock, or the sonar clock shifted onto ROS time because it is
  // unsynchronized.
  enum class TimestampSource : int
  {
    NONE,
    SENSOR,
    RECEIVE,
  };

  enum class ConfigurationState : int
  {
    DISABLED,
    PENDING,
    SUCCESSFUL,
    FAILED,
  };

  // Settings fixed for one configure/cleanup cycle.
  struct Settings
  {
    std::string sonar_ip;
    std::string fallback_ip;
    std::optional<std::uint32_t> sonar_address;
    std::optional<std::uint32_t> fallback_address;
    std::string frame_id;
    std::string imu_frame_id;
    bool use_sensor_timestamps{true};
    std::int64_t max_sensor_clock_offset_ns{0};
    conversions::ImuNoise imu_noise;
    double packet_stale_timeout{2.0};
    bool point_cloud_intensity{true};
    std::uint16_t http_port{80};
    std::chrono::milliseconds http_timeout{5000};
  };

  // A range image waiting for its signal-strength image.
  struct CloudContext
  {
    std_msgs::msg::Header header;
    std::chrono::steady_clock::time_point arrived;
  };
  using CloudPairing = RangeSignalPairing<CloudContext>;

  // Latest sonar HTTP API readings, written by device_status_thread_.
  struct DeviceStatus
  {
    std::optional<DeviceInfo> info;
    std::map<std::string, StatusEntry> components;
    std::optional<double> temperature;
    std::optional<TimeStatus> time;
    // Empty after a successful poll.
    std::string error;
    std::int64_t last_success_steady_ns{-1};
  };

  void configure_driver();
  void release();
  void stop_streaming();
  void stop_configuration();

  // Runs on receive_thread_ while active: blocks on the socket and publishes
  // each product as soon as its datagram arrives.
  void receive_loop(const std::stop_token & stop);
  [[nodiscard]] bool accepts_source(std::uint32_t source_address) const;
  void handle_datagram(const Datagram & datagram);
  void handle_range_image(
    protocol::RangeImage && image,
    const builtin_interfaces::msg::Time & fallback_stamp);
  void handle_bitmap_image(
    protocol::BitmapImage && image,
    const builtin_interfaces::msg::Time & fallback_stamp);
  void publish_cloud(CloudPairing::Pair && pair);
  // Publish a waiting range image without intensity once its signal image is
  // overdue; returns how long the receive thread may wait.
  [[nodiscard]] std::chrono::milliseconds release_overdue_cloud();
  void handle_imu_batch(
    const protocol::ImuBatch & batch,
    const builtin_interfaces::msg::Time & fallback_stamp);
  // Offset to add to a message's sensor timestamps: zero while the sonar clock
  // agrees with receive time, otherwise the shared estimate that maps the
  // sonar clock onto ROS time.
  [[nodiscard]] std::int64_t sensor_clock_offset(
    const std::optional<protocol::Timestamp> & reference,
    const builtin_interfaces::msg::Time & receive_stamp);
  void observe_sequence(SequenceState & state, std::uint32_t sequence_id);
  void diagnose_stream(diagnostic_updater::DiagnosticStatusWrapper & status);
  void diagnose_device(diagnostic_updater::DiagnosticStatusWrapper & status);
  void publish_imu_transform();
  void start_configuration(const ConfigurationRequest & request);
  void start_device_polling(std::chrono::milliseconds period);
  void poll_device(const std::stop_token & stop, std::chrono::milliseconds period);
  void stop_device_polling();

  std::shared_ptr<ParamListener> parameter_listener_;
  // Driver parameters may change only while unconfigured; configure applies them.
  std::unordered_set<std::string> driver_parameters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  Settings settings_;

  // Configured state.
  std::unique_ptr<UdpReceiver> receiver_;
  Publisher<sensor_msgs::msg::Image> range_image_publisher_;
  Publisher<sensor_msgs::msg::Image> intensity_image_publisher_;
  Publisher<sensor_msgs::msg::Image> shaded_image_publisher_;
  Publisher<sensor_msgs::msg::PointCloud2> point_cloud_publisher_;
  Publisher<sensor_msgs::msg::Imu> imu_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

  // Touched only by receive_thread_ (reset while it is stopped).
  SequenceState range_sequence_;
  SequenceState intensity_sequence_;
  SequenceState shaded_sequence_;
  SequenceState imu_sequence_;
  // A 10 s window follows crystal drift to well under a millisecond; a 1 s
  // jump above the estimate is treated as a sonar clock step.
  SensorClockOffset clock_offset_{10'000'000'000, 1'000'000'000};
  std::unordered_set<std::uint32_t> warned_sources_;
  CloudPairing pairing_;

  // Shared with the diagnostics timer.
  Metrics metrics_;
  std::atomic<bool> streaming_{false};
  std::atomic<std::int64_t> last_valid_packet_steady_ns_{-1};
  std::atomic<TimestampSource> timestamp_source_{TimestampSource::NONE};
  // Receive time minus sensor time for the latest timestamped message.
  std::atomic<std::int64_t> last_sensor_clock_offset_ns_{0};

  std::atomic<ConfigurationState> configuration_state_{ConfigurationState::DISABLED};
  std::mutex configuration_mutex_;
  std::string configuration_error_;
  std::vector<std::string> configuration_unsupported_;
  std::jthread configuration_thread_;

  std::mutex device_mutex_;
  DeviceStatus device_;
  bool device_polling_{false};
  std::string device_hardware_id_;
  std::jthread device_status_thread_;

  // The sonar images at 5 Hz (low frequency mode) or 20 Hz (high frequency).
  double minimum_range_rate_{5.0};
  double maximum_range_rate_{20.0};
  diagnostic_updater::FrequencyStatus range_rate_;
  diagnostic_updater::Updater diagnostics_;

  std::jthread receive_thread_;
};

}  // namespace sonar3d
