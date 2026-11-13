// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <sstream>
#include "packet_timing.hpp"
#include "sonar3d/protocol.hpp"

namespace
{
namespace protocol = sonar3d::protocol;
namespace timing = sonar3d::timing;

TEST(PacketTiming, ExtractsExactNativeTimeFromBothWireVersions)
{
  protocol::RangeImage image;
  image.header.timestamp = protocol::Timestamp{1790000000, 123456789};
  image.header.sequence_id = 23;
  for (const auto version : {protocol::ProtocolVersion::RIP1, protocol::ProtocolVersion::RIP2}) {
    const auto result = timing::describe_packet(protocol::encode_packet(image, version));
    EXPECT_EQ(result["state"], "decoded");
    EXPECT_EQ(result["measurement_ns"], 1790000000123456789LL);
    EXPECT_EQ(result["sequence_id"], 23);
    EXPECT_EQ(result["report_type"], "RangeImage");
  }
}

TEST(PacketTiming, MissingZeroAndMalformedTimesStayUnmapped)
{
  protocol::BitmapImage image;
  image.type = protocol::BitmapImageType::SHADED_IMAGE;
  EXPECT_EQ(timing::describe_packet(protocol::encode_packet(image))["state"], "missing_timestamp");
  for (const auto stamp : {protocol::Timestamp{0, 0}, protocol::Timestamp{-1, 0},
      protocol::Timestamp{1, -1}, protocol::Timestamp{1, 1000000000},
      protocol::Timestamp{2147483648LL, 0}})
  {
    image.header.timestamp = stamp;
    const auto result = timing::describe_packet(protocol::encode_packet(image));
    EXPECT_EQ(result["state"], "invalid_timestamp");
    EXPECT_TRUE(result["measurement_ns"].is_null());
    EXPECT_EQ(result["image_type"], 1);
  }
}

TEST(PacketTiming, PrivateOrientationRetainsItsOwnTimestampOnly)
{
  protocol::ImuOrientation orientation;
  orientation.header.timestamp = protocol::Timestamp{1790000000, 42};
  orientation.source_time_2 = 777;
  const auto result = timing::describe_packet(protocol::encode_packet(orientation));
  EXPECT_EQ(result["report_type"], "ImuOrientation");
  EXPECT_EQ(result["measurement_ns"], 1790000000000000042LL);
  EXPECT_FALSE(result.contains("source_time_2"));  // Opaque fields never become clock anchors.
}

TEST(PacketTiming, BatchUsesFirstSampleAndReportsTheWholeSampleRange)
{
  protocol::ImuBatch batch;
  batch.sample_count = 2;
  batch.timestamps = {{100, 1}, {100, 123456}};
  batch.specific_force.resize(6);
  batch.rate_of_turn.resize(6);
  const auto result = timing::describe_packet(protocol::encode_packet(batch));
  EXPECT_EQ(result["state"], "decoded");
  EXPECT_EQ(result["measurement_ns"], 100000000001LL);
  EXPECT_EQ(result["sample_max_ns"], 100000123456LL);
  batch.timestamps.back() = {99, 1};
  EXPECT_EQ(timing::describe_packet(protocol::encode_packet(batch))["state"], "invalid_batch");
  batch.timestamps.back() = {0, 0};
  EXPECT_TRUE(timing::describe_packet(protocol::encode_packet(batch))["measurement_ns"].is_null());
  batch.timestamps.back() = {100, 2};
  batch.specific_force.pop_back();
  EXPECT_EQ(timing::describe_packet(protocol::encode_packet(batch))["state"], "invalid_batch");
}

TEST(PacketTiming, DamagedPacketsCannotSupplyClocks)
{
  protocol::RangeImage image;
  image.header.timestamp = protocol::Timestamp{100, 1};
  auto bytes = protocol::encode_packet(image);
  bytes.back() ^= 1;
  const auto result = timing::describe_packet(bytes);
  EXPECT_EQ(result["state"], "invalid_packet");
  EXPECT_TRUE(result["measurement_ns"].is_null());
  EXPECT_TRUE(result.contains("protocol_error"));
}

std::string request(const std::vector<std::uint8_t> & bytes)
{
  const auto size = static_cast<std::uint32_t>(bytes.size());
  std::string result;
  for (int shift = 0; shift < 32; shift += 8) {
    result += static_cast<char>(size >> shift);
  }
  result.append(reinterpret_cast<const char *>(bytes.data()), bytes.size());
  return result;
}

TEST(PacketTiming, TransportPairsEveryRequestIncludingADamagedDatagram)
{
  protocol::RangeImage image;
  image.header.timestamp = protocol::Timestamp{100, 1};
  const auto good = protocol::encode_packet(image);
  std::istringstream input(request({1, 2, 3}) + request(good));
  std::ostringstream output;
  timing::inspect_packets(input, output);
  std::istringstream lines(output.str());
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(lines, line)));
  EXPECT_EQ(nlohmann::json::parse(line)["state"], "invalid_packet");
  ASSERT_TRUE(static_cast<bool>(std::getline(lines, line)));
  const auto result = nlohmann::json::parse(line);
  EXPECT_EQ(result["input_index"], 1);
  EXPECT_EQ(result["state"], "decoded");
}

TEST(PacketTiming, TransportRejectsTruncationAndExcessiveAllocation)
{
  for (const std::string & bytes : {std::string("a"), std::string("\xff\xff\xff\xff", 4),
      std::string("\x03\0\0\0x", 5)})
  {
    std::istringstream input(bytes);
    std::ostringstream output;
    EXPECT_THROW(timing::inspect_packets(input, output), std::runtime_error);
  }
}
}  // namespace
