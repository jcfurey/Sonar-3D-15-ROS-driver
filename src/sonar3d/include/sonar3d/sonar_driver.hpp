// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_set>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "sonar3d/protocol.hpp"
#include "sonar3d/sensor_clock.hpp"
#include "sonar3d/sonar3d_parameters.hpp"
#include "sonar3d/udp_receiver.hpp"

namespace sonar3d
{

class SonarDriver final : public rclcpp::Node
{
public:
  explicit SonarDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SonarDriver() override;

private:
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

  // Runs on receive_thread_: blocks on the socket and publishes each product
  // as soon as its datagram arrives, independent of executor scheduling.
  void receive_loop(const std::stop_token & stop);
  [[nodiscard]] bool accepts_source(std::uint32_t source_address) const;
  void handle_datagram(const Datagram & datagram);
  void handle_range_image(
    const protocol::RangeImage & image,
    const builtin_interfaces::msg::Time & fallback_stamp);
  void handle_bitmap_image(
    const protocol::BitmapImage & image,
    const builtin_interfaces::msg::Time & fallback_stamp);
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
  void publish_diagnostics();
  void publish_imu_transform();
  void start_configuration(double speed_of_sound, double timeout_seconds);

  std::shared_ptr<ParamListener> parameter_listener_;
  std::string sonar_ip_;
  std::string fallback_ip_;
  std::optional<std::uint32_t> sonar_address_;
  std::optional<std::uint32_t> fallback_address_;
  std::string frame_id_;
  std::string imu_frame_id_;
  bool use_sensor_timestamps_{true};
  std::int64_t max_sensor_clock_offset_ns_{0};
  bool publish_point_cloud_{true};
  bool publish_range_image_{true};
  bool publish_bitmap_images_{true};
  bool publish_imu_{true};
  double packet_stale_timeout_{2.0};

  std::unique_ptr<UdpReceiver> receiver_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr range_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr shaded_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Touched only by receive_thread_.
  SequenceState range_sequence_;
  SequenceState intensity_sequence_;
  SequenceState shaded_sequence_;
  SequenceState imu_sequence_;
  // A 10 s window follows crystal drift to well under a millisecond; a 1 s
  // jump above the estimate is treated as a sonar clock step.
  SensorClockOffset clock_offset_{10'000'000'000, 1'000'000'000};
  std::unordered_set<std::uint32_t> warned_sources_;

  // Shared with the diagnostics timer.
  Metrics metrics_;
  std::atomic<std::int64_t> last_valid_packet_steady_ns_{-1};
  std::atomic<TimestampSource> timestamp_source_{TimestampSource::NONE};
  // Receive time minus sensor time for the latest timestamped message.
  std::atomic<std::int64_t> last_sensor_clock_offset_ns_{0};

  // 0 = configuration disabled, 1 = pending, 2 = successful, 3 = failed.
  std::atomic<int> configuration_state_{0};
  std::mutex configuration_error_mutex_;
  std::string configuration_error_;
  std::jthread configuration_thread_;
  std::jthread receive_thread_;
};

}  // namespace sonar3d
