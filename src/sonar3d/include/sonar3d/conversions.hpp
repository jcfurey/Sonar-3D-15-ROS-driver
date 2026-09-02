// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "sonar3d/protocol.hpp"

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

[[nodiscard]] std::vector<float> range_image_to_meters(const protocol::RangeImage & image);
[[nodiscard]] std::vector<RangePoint> range_image_to_points(const protocol::RangeImage & image);

[[nodiscard]] std_msgs::msg::Header make_header(
  const protocol::MessageHeader & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp = true);

[[nodiscard]] sensor_msgs::msg::Image make_range_image(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header);

[[nodiscard]] sensor_msgs::msg::PointCloud2 make_point_cloud(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header);

[[nodiscard]] sensor_msgs::msg::Image make_bitmap_image(
  const protocol::BitmapImage & source,
  const std_msgs::msg::Header & header);

[[nodiscard]] std::vector<sensor_msgs::msg::Imu> make_imu_messages(
  const protocol::ImuBatch & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp = true);

}  // namespace sonar3d::conversions
