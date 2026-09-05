// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "sonar3d/conversions.hpp"

#include <numbers>
#include <bit>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>

namespace sonar3d::conversions
{
namespace
{

[[nodiscard]] std::size_t expected_pixel_count(
  std::uint32_t width,
  std::uint32_t height,
  std::size_t actual,
  const char * image_name)
{
  if (width == 0 || height == 0) {
    throw std::invalid_argument(std::string(image_name) + " has a zero width or height");
  }
  const auto expected = static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
  if (expected > std::numeric_limits<std::size_t>::max()) {
    throw std::length_error(std::string(image_name) + " dimensions exceed host limits");
  }
  if (actual != static_cast<std::size_t>(expected)) {
    throw std::invalid_argument(
      std::string(image_name) + " pixel count does not match width times height");
  }
  return static_cast<std::size_t>(expected);
}

void validate_range_image(const protocol::RangeImage & image, bool require_angles)
{
  static_cast<void>(expected_pixel_count(image.width, image.height, image.pixels.size(),
        "range image"));
  if (!std::isfinite(image.pixel_scale) || image.pixel_scale <= 0.0F) {
    throw std::invalid_argument("range image pixel scale must be finite and positive");
  }
  if (require_angles &&
    (!std::isfinite(image.horizontal_fov_degrees) ||
    !std::isfinite(image.vertical_fov_degrees) ||
    image.horizontal_fov_degrees < 0.0F || image.horizontal_fov_degrees > 360.0F ||
    image.vertical_fov_degrees < 0.0F || image.vertical_fov_degrees > 360.0F))
  {
    throw std::invalid_argument(
          "range image fields of view must be finite and between 0 and 360 degrees");
  }
}

[[nodiscard]] builtin_interfaces::msg::Time choose_stamp(
  const std::optional<protocol::Timestamp> & sensor_stamp,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp)
{
  if (!use_sensor_timestamp || !sensor_stamp ||
    (sensor_stamp->seconds == 0 && sensor_stamp->nanoseconds == 0))
  {
    return fallback_stamp;
  }
  if (sensor_stamp->nanoseconds < 0 || sensor_stamp->nanoseconds >= 1'000'000'000) {
    throw std::invalid_argument("sonar timestamp nanoseconds are outside [0, 1e9)");
  }
  if (sensor_stamp->seconds < std::numeric_limits<std::int32_t>::min() ||
    sensor_stamp->seconds > std::numeric_limits<std::int32_t>::max())
  {
    throw std::invalid_argument("sonar timestamp seconds are outside the ROS message range");
  }

  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(sensor_stamp->seconds);
  result.nanosec = static_cast<std::uint32_t>(sensor_stamp->nanoseconds);
  return result;
}

[[nodiscard]] float pixel_angle(
  float field_of_view_radians, std::uint32_t index,
  std::uint32_t count)
{
  if (count == 1) {
    return 0.0F;
  }
  return (static_cast<float>(index) / static_cast<float>(count - 1U) - 0.5F) *
         field_of_view_radians;
}

[[nodiscard]] sensor_msgs::msg::PointField point_field(
  const std::string & name,
  std::uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}

template<typename Output>
void for_each_range_point(const protocol::RangeImage & image, Output output)
{
  const auto horizontal_fov =
    image.horizontal_fov_degrees * std::numbers::pi_v<float>/ 180.0F;
  const auto vertical_fov = image.vertical_fov_degrees * std::numbers::pi_v<float>/ 180.0F;
  struct Column
  {
    float yaw;
    float cosine;
    float sine;
  };
  // Column geometry is identical on every row. Compute the trigonometry once
  // per axis, retaining the original order of the per-point multiplications.
  std::vector<Column> columns;
  columns.reserve(image.width);
  for (std::uint32_t column = 0; column < image.width; ++column) {
    const auto yaw = pixel_angle(horizontal_fov, column, image.width);
    columns.push_back({yaw, std::cos(yaw), std::sin(yaw)});
  }
  for (std::uint32_t row = 0; row < image.height; ++row) {
    const auto pitch = pixel_angle(vertical_fov, row, image.height);
    const auto cosine_pitch = std::cos(pitch);
    const auto sine_pitch = std::sin(pitch);
    for (std::uint32_t column = 0; column < image.width; ++column) {
      const auto pixel = image.pixels[static_cast<std::size_t>(row) * image.width + column];
      if (pixel == 0) {
        continue;
      }
      const auto distance = static_cast<float>(pixel) * image.pixel_scale;
      if (!std::isfinite(distance)) {
        throw std::invalid_argument("range image produces a non-finite point distance");
      }
      const auto & direction = columns[column];
      output(RangePoint{
            distance * cosine_pitch * direction.cosine,
            -distance * cosine_pitch * direction.sine,
            distance * sine_pitch,
            distance,
            -direction.yaw,
            pitch,
        });
    }
  }
}

}  // namespace

std::vector<float> range_image_to_meters(const protocol::RangeImage & image)
{
  validate_range_image(image, false);
  std::vector<float> ranges;
  ranges.reserve(image.pixels.size());
  for (const auto pixel : image.pixels) {
    const auto range = static_cast<float>(pixel) * image.pixel_scale;
    if (!std::isfinite(range)) {
      throw std::invalid_argument("range image produces a non-finite distance");
    }
    ranges.push_back(range);
  }
  return ranges;
}

std::vector<RangePoint> range_image_to_points(const protocol::RangeImage & image)
{
  validate_range_image(image, true);
  std::vector<RangePoint> points;
  points.reserve(image.pixels.size());
  for_each_range_point(image, [&points](const RangePoint & point) {points.push_back(point);});
  return points;
}

std_msgs::msg::Header make_header(
  const protocol::MessageHeader & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp)
{
  if (frame_id.empty()) {
    throw std::invalid_argument("frame_id must not be empty");
  }
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = choose_stamp(source.timestamp, fallback_stamp, use_sensor_timestamp);
  return header;
}

sensor_msgs::msg::Image make_range_image(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header)
{
  validate_range_image(source, false);
  sensor_msgs::msg::Image message;
  message.header = header;
  message.width = source.width;
  message.height = source.height;
  message.encoding = "32FC1";
  message.is_bigendian = std::endian::native == std::endian::big;
  message.step = source.width * static_cast<std::uint32_t>(sizeof(float));
  message.data.resize(source.pixels.size() * sizeof(float));
  for (std::size_t index = 0; index < source.pixels.size(); ++index) {
    const auto range = static_cast<float>(source.pixels[index]) * source.pixel_scale;
    if (!std::isfinite(range)) {
      throw std::invalid_argument("range image produces a non-finite distance");
    }
    std::memcpy(message.data.data() + index * sizeof(float), &range, sizeof(float));
  }
  return message;
}

sensor_msgs::msg::PointCloud2 make_point_cloud(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header)
{
  constexpr std::uint32_t kFieldCount = 6;
  constexpr std::uint32_t kPointStep = kFieldCount * sizeof(float);
  validate_range_image(source, true);
  const auto point_count = static_cast<std::size_t>(std::count_if(
      source.pixels.begin(), source.pixels.end(), [](std::uint32_t pixel) {return pixel != 0;}));
  if (point_count > std::numeric_limits<std::uint32_t>::max()) {
    throw std::length_error("point cloud contains too many points for PointCloud2");
  }

  sensor_msgs::msg::PointCloud2 message;
  message.header = header;
  message.height = 1;
  message.width = static_cast<std::uint32_t>(point_count);
  message.fields = {
    point_field("x", 0),
    point_field("y", 4),
    point_field("z", 8),
    point_field("range", 12),
    point_field("azimuth", 16),
    point_field("elevation", 20),
  };
  message.is_bigendian = std::endian::native == std::endian::big;
  message.point_step = kPointStep;
  message.row_step = message.point_step * message.width;
  message.is_dense = true;
  message.data.resize(static_cast<std::size_t>(message.row_step));

  if (point_count != 0) {
    std::size_t offset = 0;
    for_each_range_point(source, [&message, &offset](const RangePoint & point) {
        const std::array<float, kFieldCount> values{
          point.x, point.y, point.z, point.range, point.azimuth, point.elevation,
        };
        std::memcpy(message.data.data() + offset, values.data(), kPointStep);
        offset += kPointStep;
      });
  }
  return message;
}

sensor_msgs::msg::Image make_bitmap_image(
  const protocol::BitmapImage & source,
  const std_msgs::msg::Header & header)
{
  static_cast<void>(
    expected_pixel_count(source.width, source.height, source.pixels.size(), "bitmap image"));
  sensor_msgs::msg::Image message;
  message.header = header;
  message.width = source.width;
  message.height = source.height;
  message.encoding = "mono8";
  message.is_bigendian = false;
  message.step = source.width;
  message.data = source.pixels;
  return message;
}

std::vector<sensor_msgs::msg::Imu> make_imu_messages(
  const protocol::ImuBatch & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp)
{
  const auto sample_count = static_cast<std::size_t>(source.sample_count);
  if (source.timestamps.size() != sample_count ||
    source.specific_force.size() != sample_count * 3U ||
    source.rate_of_turn.size() != sample_count * 3U)
  {
    throw std::invalid_argument("IMU batch array sizes do not match its sample count");
  }

  std::vector<sensor_msgs::msg::Imu> messages;
  messages.reserve(sample_count);
  for (std::size_t index = 0; index < sample_count; ++index) {
    protocol::MessageHeader source_header;
    source_header.timestamp = source.timestamps[index];

    sensor_msgs::msg::Imu message;
    message.header = make_header(source_header, frame_id, fallback_stamp, use_sensor_timestamp);
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;

    // Water Linked reports x-forward, y-right, z-down. A 180-degree rotation
    // about x maps both vectors into the ROS body convention (x-forward,
    // y-left, z-up).
    message.linear_acceleration.x = source.specific_force[index * 3U];
    message.linear_acceleration.y = -source.specific_force[index * 3U + 1U];
    message.linear_acceleration.z = -source.specific_force[index * 3U + 2U];
    message.angular_velocity.x = source.rate_of_turn[index * 3U];
    message.angular_velocity.y = -source.rate_of_turn[index * 3U + 1U];
    message.angular_velocity.z = -source.rate_of_turn[index * 3U + 2U];
    messages.push_back(std::move(message));
  }
  return messages;
}

}  // namespace sonar3d::conversions
