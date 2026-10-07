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

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;

[[nodiscard]] builtin_interfaces::msg::Time choose_stamp(
  const std::optional<protocol::Timestamp> & sensor_stamp,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp,
  std::int64_t sensor_offset_nanoseconds)
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
  if (sensor_offset_nanoseconds == 0) {
    result.sec = static_cast<std::int32_t>(sensor_stamp->seconds);
    result.nanosec = static_cast<std::uint32_t>(sensor_stamp->nanoseconds);
    return result;
  }

  // Validated above: seconds fit int32, so the product cannot overflow int64.
  const auto sensor = sensor_stamp->seconds * kNanosecondsPerSecond + sensor_stamp->nanoseconds;
  std::int64_t shifted{};
  if (__builtin_add_overflow(sensor, sensor_offset_nanoseconds, &shifted) || shifted < 0 ||
    shifted / kNanosecondsPerSecond > std::numeric_limits<std::int32_t>::max())
  {
    throw std::invalid_argument("re-anchored sonar timestamp is outside the ROS message range");
  }
  result.sec = static_cast<std::int32_t>(shifted / kNanosecondsPerSecond);
  result.nanosec = static_cast<std::uint32_t>(shifted % kNanosecondsPerSecond);
  return result;
}

// Vendor pixel rows run from the lowest elevation (pitch = -fovV/2) upward.
// sensor_msgs/Image rows run top-down, so published row r is vendor row
// height - 1 - r: the images appear upright and share one pixel grid.
[[nodiscard]] std::size_t vendor_row_offset(
  std::uint32_t width, std::uint32_t height,
  std::uint32_t image_row)
{
  return static_cast<std::size_t>(height - 1U - image_row) * width;
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
      const auto index = static_cast<std::size_t>(row) * image.width + column;
      const auto pixel = image.pixels[index];
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
        }, index);
    }
  }
}

// Water Linked encodes signal strength as pixel = 100 log10(strength / 30);
// zero means no return. Decoded as by wlsonar's strength_linear helper.
[[nodiscard]] const std::array<float, 256> & linear_strength_table()
{
  static const auto table = [] {
      std::array<float, 256> values{};
      for (std::size_t pixel = 1; pixel < values.size(); ++pixel) {
        values[pixel] = static_cast<float>(
          std::round(30.0 * std::pow(10.0, static_cast<double>(pixel) / 100.0)));
      }
      return values;
    }();
  return table;
}

// A compile-time layout keeps the per-point copy a fixed-size store.
template<bool kIntensity>
void fill_cloud(
  const protocol::RangeImage & source, const protocol::BitmapImage * signal,
  std::uint8_t * output)
{
  constexpr std::size_t kFieldCount = kIntensity ? 7U : 6U;
  const auto & strength = linear_strength_table();
  for_each_range_point(
    source, [&](const RangePoint & point, std::size_t index) {
      std::array<float, kFieldCount> values{};
      if constexpr (kIntensity) {
        // Both images share the vendor pixel grid; no signal means unknown.
        const auto intensity = signal != nullptr ? strength[signal->pixels[index]] : 0.0F;
        values = {point.x, point.y, point.z, intensity, point.range, point.azimuth,
          point.elevation};
      } else {
        values = {point.x, point.y, point.z, point.range, point.azimuth, point.elevation};
      }
      std::memcpy(output, values.data(), sizeof(values));
      output += sizeof(values);
    });
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
  for_each_range_point(
    image, [&points](const RangePoint & point, std::size_t) {points.push_back(point);});
  return points;
}

std::optional<std::int64_t> sensor_nanoseconds(const std::optional<protocol::Timestamp> & stamp)
{
  if (!stamp || (stamp->seconds == 0 && stamp->nanoseconds == 0) ||
    stamp->nanoseconds < 0 || stamp->nanoseconds >= 1'000'000'000 ||
    stamp->seconds < std::numeric_limits<std::int32_t>::min() ||
    stamp->seconds > std::numeric_limits<std::int32_t>::max())
  {
    return std::nullopt;
  }
  return stamp->seconds * kNanosecondsPerSecond + stamp->nanoseconds;
}

std_msgs::msg::Header make_header(
  const protocol::MessageHeader & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp,
  std::int64_t sensor_offset_nanoseconds)
{
  if (frame_id.empty()) {
    throw std::invalid_argument("frame_id must not be empty");
  }
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = choose_stamp(
    source.timestamp, fallback_stamp, use_sensor_timestamp, sensor_offset_nanoseconds);
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
  auto * output = message.data.data();
  for (std::uint32_t row = 0; row < source.height; ++row) {
    const auto * input = source.pixels.data() + vendor_row_offset(source.width, source.height, row);
    for (std::uint32_t column = 0; column < source.width; ++column) {
      const auto range = static_cast<float>(input[column]) * source.pixel_scale;
      if (!std::isfinite(range)) {
        throw std::invalid_argument("range image produces a non-finite distance");
      }
      std::memcpy(output, &range, sizeof(float));
      output += sizeof(float);
    }
  }
  return message;
}

void validate_point_cloud_source(const protocol::RangeImage & image)
{
  validate_range_image(image, true);
}

float linear_signal_strength(std::uint8_t pixel)
{
  return linear_strength_table()[pixel];
}

sensor_msgs::msg::PointCloud2 make_point_cloud(
  const protocol::RangeImage & source,
  const std_msgs::msg::Header & header,
  bool with_intensity,
  const protocol::BitmapImage * signal)
{
  validate_range_image(source, true);
  if (signal != nullptr) {
    if (!with_intensity) {
      throw std::invalid_argument("a signal image needs the intensity field");
    }
    if (signal->type != protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE ||
      signal->width != source.width || signal->height != source.height ||
      signal->pixels.size() != source.pixels.size())
    {
      throw std::invalid_argument("signal-strength image does not match the range image");
    }
  }
  const std::uint32_t field_count = with_intensity ? 7U : 6U;
  const auto point_step = field_count * static_cast<std::uint32_t>(sizeof(float));
  const auto point_count = static_cast<std::size_t>(std::count_if(
      source.pixels.begin(), source.pixels.end(), [](std::uint32_t pixel) {return pixel != 0;}));
  if (point_count > std::numeric_limits<std::uint32_t>::max() / point_step) {
    throw std::length_error("point cloud contains too many points for PointCloud2");
  }

  sensor_msgs::msg::PointCloud2 message;
  message.header = header;
  message.height = 1;
  message.width = static_cast<std::uint32_t>(point_count);
  if (with_intensity) {
    message.fields = {
      point_field("x", 0), point_field("y", 4), point_field("z", 8),
      point_field("intensity", 12), point_field("range", 16), point_field("azimuth", 20),
      point_field("elevation", 24),
    };
  } else {
    message.fields = {
      point_field("x", 0), point_field("y", 4), point_field("z", 8),
      point_field("range", 12), point_field("azimuth", 16), point_field("elevation", 20),
    };
  }
  message.is_bigendian = std::endian::native == std::endian::big;
  message.point_step = point_step;
  message.row_step = message.point_step * message.width;
  message.is_dense = true;
  message.data.resize(static_cast<std::size_t>(message.row_step));

  if (point_count != 0 && with_intensity) {
    fill_cloud<true>(source, signal, message.data.data());
  } else if (point_count != 0) {
    fill_cloud<false>(source, signal, message.data.data());
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
  message.data.resize(source.pixels.size());
  for (std::uint32_t row = 0; row < source.height; ++row) {
    std::memcpy(
      message.data.data() + static_cast<std::size_t>(row) * source.width,
      source.pixels.data() + vendor_row_offset(source.width, source.height, row),
      source.width);
  }
  return message;
}

std::vector<sensor_msgs::msg::Imu> make_imu_messages(
  const protocol::ImuBatch & source,
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & fallback_stamp,
  bool use_sensor_timestamp,
  std::int64_t sensor_offset_nanoseconds,
  const ImuNoise & noise)
{
  if (!std::isfinite(noise.linear_acceleration_stddev) || noise.linear_acceleration_stddev < 0.0 ||
    !std::isfinite(noise.angular_velocity_stddev) || noise.angular_velocity_stddev < 0.0)
  {
    throw std::invalid_argument("IMU standard deviations must be finite and non-negative");
  }
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
    message.header = make_header(
      source_header, frame_id, fallback_stamp, use_sensor_timestamp, sensor_offset_nanoseconds);
    message.orientation.w = 1.0;
    // REP-145: -1 marks the orientation as not reported.
    message.orientation_covariance[0] = -1.0;
    for (const std::size_t diagonal : {0U, 4U, 8U}) {
      message.linear_acceleration_covariance[diagonal] =
        noise.linear_acceleration_stddev * noise.linear_acceleration_stddev;
      message.angular_velocity_covariance[diagonal] =
        noise.angular_velocity_stddev * noise.angular_velocity_stddev;
    }

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

namespace
{
template<typename Image>
msg::ImageMetadata image_metadata(const Image & source, const std_msgs::msg::Header & header)
{
  static_cast<void>(expected_pixel_count(source.width, source.height, source.pixels.size(),
        "image metadata"));
  msg::ImageMetadata metadata;
  metadata.header = header;
  metadata.sequence_id = source.header.sequence_id;
  metadata.width = source.width;
  metadata.height = source.height;
  metadata.horizontal_fov_degrees = source.horizontal_fov_degrees;
  metadata.vertical_fov_degrees = source.vertical_fov_degrees;
  metadata.configured_range_m = source.range;
  metadata.speed_of_sound_mps = source.speed_of_sound;
  metadata.frequency = source.frequency;
  metadata.sensor_timestamp_valid = sensor_nanoseconds(source.header.timestamp).has_value();
  if (source.header.timestamp) {
    metadata.sensor_stamp_seconds = source.header.timestamp->seconds;
    metadata.sensor_stamp_nanoseconds = source.header.timestamp->nanoseconds;
  }
  metadata.driver_version = SONAR3D_VERSION;
  metadata.driver_source_sha256 = SONAR3D_SOURCE_SHA256;
  return metadata;
}
}  // namespace

msg::ImageMetadata make_image_metadata(
  const protocol::RangeImage & source, const std_msgs::msg::Header & header)
{
  auto metadata = image_metadata(source, header);
  metadata.image_type = msg::ImageMetadata::RANGE;
  metadata.range_pixel_scale_m = source.pixel_scale;
  return metadata;
}

msg::ImageMetadata make_image_metadata(
  const protocol::BitmapImage & source, const std_msgs::msg::Header & header)
{
  auto metadata = image_metadata(source, header);
  metadata.image_type = source.type == protocol::BitmapImageType::SHADED_IMAGE ?
    msg::ImageMetadata::SHADED : msg::ImageMetadata::SIGNAL;
  return metadata;
}


}  // namespace sonar3d::conversions
