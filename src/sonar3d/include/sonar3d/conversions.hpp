// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "sonar3d/protocol.hpp"
#include "sonar3d/msg/image_metadata.hpp"

namespace sonar3d::conversions
{

struct RangePoint
{
  float x{};
  float y{};
  float z{};
  float range{};
  float azimuth{};
  float elevation{};

  bool operator==(const RangePoint &) const = default;
};

// Ranges in the vendor's pixel order (row 0 is the lowest elevation).
[[nodiscard]] std::vector<float> range_image_to_meters(const protocol::RangeImage & image);
[[nodiscard]] std::vector<RangePoint> range_image_to_points(const protocol::RangeImage & image);

// Nanoseconds since the epoch for a usable sensor timestamp. Absent, zero, and
// out-of-range timestamps return nullopt; this never throws.
[[nodiscard]] std::optional<std::int64_t> sensor_nanoseconds(
  const std::optional<protocol::Timestamp> & stamp);

// sensor_offset_nanoseconds is added to a valid sensor timestamp. A driver uses
// it to map an unsynchronized sonar clock onto ROS time.
[[nodiscard]] std_msgs::msg::Header make_header(
  const protocol::MessageHeader & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp = true,
  std::int64_t sensor_offset_nanoseconds = 0);

// ROS images are upright: row 0 is the highest elevation, so published row r
// holds vendor row height - 1 - r. Columns keep the vendor's left-to-right order.
[[nodiscard]] sensor_msgs::msg::Image make_range_image(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header);

// Header exactly matches its image; original sensor clock remains separate.
[[nodiscard]] msg::ImageMetadata make_image_metadata(
  const protocol::RangeImage & source, const std_msgs::msg::Header & header);
[[nodiscard]] msg::ImageMetadata make_image_metadata(
  const protocol::BitmapImage & source, const std_msgs::msg::Header & header);

// Throws std::invalid_argument unless the image can be converted to a point
// cloud; lets callers that defer the cloud reject a malformed image at once.
void validate_point_cloud_source(const protocol::RangeImage & image);

// Linear signal strength (about 30 to 10600) for a SIGNAL_STRENGTH_IMAGE
// pixel, which Water Linked encodes as 100 log10(strength / 30); zero is no
// return.
[[nodiscard]] float linear_signal_strength(std::uint8_t pixel);

// Valid returns as x, y, z, [intensity,] range, azimuth, elevation float32
// fields. with_intensity adds linear signal strength from the same shot's
// SIGNAL_STRENGTH_IMAGE; without that image (lost or late) intensity is 0.
[[nodiscard]] sensor_msgs::msg::PointCloud2 make_point_cloud(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header,
  bool with_intensity = false,
  const protocol::BitmapImage * signal = nullptr);

[[nodiscard]] sensor_msgs::msg::Image make_bitmap_image(
  const protocol::BitmapImage & source,
  const std_msgs::msg::Header & header);

// REP-145 noise parameters. Zero leaves a covariance at all zeros, meaning
// "unknown"; otherwise its diagonal is the squared standard deviation.
struct ImuNoise
{
  double linear_acceleration_stddev{0.0};
  double angular_velocity_stddev{0.0};
};

[[nodiscard]] std::vector<sensor_msgs::msg::Imu> make_imu_messages(
  const protocol::ImuBatch & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp = true,
  std::int64_t sensor_offset_nanoseconds = 0,
  const ImuNoise & noise = {});

}  // namespace sonar3d::conversions
