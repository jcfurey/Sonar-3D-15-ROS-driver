// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <numbers>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/protocol.hpp"

namespace
{

sonar3d::protocol::RangeImage test_range_image()
{
  sonar3d::protocol::RangeImage image;
  image.header.timestamp = sonar3d::protocol::Timestamp{123, 456};
  image.width = 3;
  image.height = 2;
  image.horizontal_fov_degrees = 90.0F;
  image.vertical_fov_degrees = 40.0F;
  image.pixel_scale = 0.01F;
  image.pixels = {100, 0, 200, 300, 400, 500};
  return image;
}

float read_float(const std::vector<std::uint8_t> & bytes, std::size_t offset)
{
  float result{};
  std::memcpy(&result, bytes.data() + offset, sizeof(result));
  return result;
}

TEST(Conversions, ScalesRangeImageToMeters)
{
  const auto ranges = sonar3d::conversions::range_image_to_meters(test_range_image());

  ASSERT_EQ(ranges.size(), 6U);
  EXPECT_FLOAT_EQ(ranges[0], 1.0F);
  EXPECT_FLOAT_EQ(ranges[1], 0.0F);
  EXPECT_FLOAT_EQ(ranges[5], 5.0F);
}

TEST(Conversions, ProducesRep103PointCloudGeometry)
{
  const auto points = sonar3d::conversions::range_image_to_points(test_range_image());

  ASSERT_EQ(points.size(), 5U);
  const auto & first = points.front();
  EXPECT_NEAR(first.range, 1.0F, 1.0e-6F);
  EXPECT_NEAR(first.azimuth, std::numbers::pi_v<float>/ 4.0F, 1.0e-6F);
  EXPECT_NEAR(first.elevation, -std::numbers::pi_v<float>/ 9.0F, 1.0e-6F);
  EXPECT_GT(first.x, 0.0F);
  EXPECT_GT(first.y, 0.0F);
  EXPECT_LT(first.z, 0.0F);
}

TEST(Conversions, BuildsRangeImageAndPointCloudContracts)
{
  builtin_interfaces::msg::Time fallback;
  fallback.sec = 9;
  const auto source = test_range_image();
  const auto header = sonar3d::conversions::make_header(source.header, "sonar_link", fallback);

  const auto image = sonar3d::conversions::make_range_image(source, header);
  EXPECT_EQ(image.header.stamp.sec, 123);
  EXPECT_EQ(image.header.stamp.nanosec, 456U);
  EXPECT_EQ(image.encoding, "32FC1");
  EXPECT_EQ(image.step, 12U);
  EXPECT_FLOAT_EQ(read_float(image.data, 0), 1.0F);
  EXPECT_FLOAT_EQ(read_float(image.data, 5U * sizeof(float)), 5.0F);

  const auto cloud = sonar3d::conversions::make_point_cloud(source, header);
  EXPECT_EQ(cloud.width, 5U);
  EXPECT_EQ(cloud.point_step, 24U);
  EXPECT_EQ(cloud.row_step, 120U);
  ASSERT_EQ(cloud.fields.size(), 6U);
  EXPECT_EQ(cloud.fields[3].name, "range");
  EXPECT_EQ(cloud.fields[4].name, "azimuth");
  EXPECT_EQ(cloud.fields[5].name, "elevation");
  EXPECT_FLOAT_EQ(read_float(cloud.data, 12), 1.0F);
}

TEST(Conversions, PreservesBitmapAndFallsBackToRosClock)
{
  sonar3d::protocol::BitmapImage source;
  source.width = 2;
  source.height = 2;
  source.pixels = {0, 64, 128, 255};
  builtin_interfaces::msg::Time fallback;
  fallback.sec = 7;
  fallback.nanosec = 8;

  const auto header = sonar3d::conversions::make_header(source.header, "sonar_link", fallback);
  const auto image = sonar3d::conversions::make_bitmap_image(source, header);

  EXPECT_EQ(image.header.stamp, fallback);
  EXPECT_EQ(image.encoding, "mono8");
  EXPECT_EQ(image.step, 2U);
  EXPECT_EQ(image.data, source.pixels);
}

TEST(Conversions, TransformsImuVectorsIntoRosBodyAxes)
{
  sonar3d::protocol::ImuBatch batch;
  batch.sample_count = 1;
  batch.timestamps = {{10, 20}};
  batch.specific_force = {1.0F, 2.0F, 3.0F};
  batch.rate_of_turn = {4.0F, 5.0F, 6.0F};

  const auto messages = sonar3d::conversions::make_imu_messages(
    batch, "sonar_imu", builtin_interfaces::msg::Time{});

  ASSERT_EQ(messages.size(), 1U);
  EXPECT_EQ(messages[0].header.stamp.sec, 10);
  EXPECT_DOUBLE_EQ(messages[0].linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(messages[0].linear_acceleration.y, -2.0);
  EXPECT_DOUBLE_EQ(messages[0].linear_acceleration.z, -3.0);
  EXPECT_DOUBLE_EQ(messages[0].angular_velocity.x, 4.0);
  EXPECT_DOUBLE_EQ(messages[0].angular_velocity.y, -5.0);
  EXPECT_DOUBLE_EQ(messages[0].angular_velocity.z, -6.0);
  EXPECT_DOUBLE_EQ(messages[0].orientation_covariance[0], -1.0);
}

TEST(Conversions, RejectsMalformedImageAndImuDimensions)
{
  auto range = test_range_image();
  range.pixels.pop_back();
  EXPECT_THROW(
    static_cast<void>(sonar3d::conversions::range_image_to_meters(range)),
    std::invalid_argument);

  sonar3d::protocol::ImuBatch imu;
  imu.sample_count = 1;
  EXPECT_THROW(
    static_cast<void>(sonar3d::conversions::make_imu_messages(
      imu, "imu", builtin_interfaces::msg::Time{})),
    std::invalid_argument);
}

}  // namespace
