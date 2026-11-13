// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "packet_timing.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "sonar3d/protocol.hpp"

namespace sonar3d::timing
{
namespace
{
std::optional<std::int64_t> nanoseconds(const protocol::Timestamp & stamp)
{
  // Match the driver's supported ROS timestamp range, without float conversion.
  if (stamp.seconds < 0 || stamp.seconds > std::numeric_limits<std::int32_t>::max() ||
    stamp.nanoseconds < 0 || stamp.nanoseconds >= 1'000'000'000 ||
    (stamp.seconds == 0 && stamp.nanoseconds == 0)) {return std::nullopt;}
  return stamp.seconds * 1'000'000'000LL + stamp.nanoseconds;
}

void describe_header(nlohmann::json & result, const protocol::MessageHeader & header)
{
  result["sequence_id"] = header.sequence_id;
  result["measurement_field"] = "header.timestamp";
  if (!header.timestamp) {
    result["state"] = "missing_timestamp";
    return;
  }
  result["native_seconds"] = header.timestamp->seconds;
  result["native_nanoseconds"] = header.timestamp->nanoseconds;
  const auto stamp = nanoseconds(*header.timestamp);
  result["state"] = stamp ? "decoded" : "invalid_timestamp";
  if (stamp) {result["measurement_ns"] = *stamp;}
}
}  // namespace

nlohmann::json describe_packet(std::span<const std::uint8_t> bytes)
{
  nlohmann::json result{{"schema_version", 1}, {"state", "invalid_packet"},
    {"measurement_ns", nullptr}, {"measurement_field", nullptr}, {"report_type", nullptr}};
  try {
    const auto packet = protocol::decode_packet(bytes);
    result["protocol"] = packet.version == protocol::ProtocolVersion::RIP1 ? "RIP1" : "RIP2";
    std::visit([&](const auto & message) {
        using T = std::decay_t<decltype(message)>;
        if constexpr (std::is_same_v<T, protocol::UnknownMessage>) {
          result["report_type"] = message.type_url;
          result["state"] = "unknown_message";
        } else if constexpr (std::is_same_v<T, protocol::ImuBatch>) {
          result["report_type"] = "ImuBatch";
          result["sequence_id"] = message.sequence_id;
          result["measurement_field"] = "timestamps[0]";
          result["sample_count"] = message.sample_count;
          result["timestamp_count"] = message.timestamps.size();
          result["invalid_samples"] = 0;
          result["sample_regressions"] = 0;
          result["sample_min_ns"] = nullptr;
          result["sample_max_ns"] = nullptr;
          std::optional<std::int64_t> first, previous, low, high;
          for (std::size_t index = 0; index < message.timestamps.size(); ++index) {
            const auto stamp = nanoseconds(message.timestamps[index]);
            if (!stamp) {
              result["invalid_samples"] = result["invalid_samples"].get<std::size_t>() + 1;
              continue;
            }
            if (index == 0) {first = stamp;}
            if (previous && *stamp < *previous) {
              result["sample_regressions"] = result["sample_regressions"].get<std::size_t>() + 1;
            }
            previous = stamp;
            low = low ? std::min(*low, *stamp) : *stamp;
            high = high ? std::max(*high, *stamp) : *stamp;
          }
          if (low) {result["sample_min_ns"] = *low;}
          if (high) {result["sample_max_ns"] = *high;}
          const auto count = message.timestamps.size();
          const bool valid = first && count == message.sample_count &&
          message.specific_force.size() == count * 3 && message.rate_of_turn.size() == count * 3 &&
          result["invalid_samples"] == 0 && result["sample_regressions"] == 0;
          result["state"] = valid ? "decoded" : "invalid_batch";
        // A raw batch has one bag event; its other sample clocks remain in the
        // original packet. Do not pretend its samples were acquired together.
          if (valid) {result["measurement_ns"] = *first;}
        } else {
          if constexpr (std::is_same_v<T, protocol::RangeImage>) {
            result["report_type"] = "RangeImage";
          } else if constexpr (std::is_same_v<T, protocol::BitmapImage>) {
            result["report_type"] = "BitmapImage";
            result["image_type"] = static_cast<std::int32_t>(message.type);
          } else {
            result["report_type"] = "ImuOrientation";
          }
          describe_header(result, message.header);
        }
    }, packet.message);
  } catch (const protocol::ProtocolError & error) {
    result["error"] = error.what();
    result["protocol_error"] = static_cast<int>(error.code());
  }
  return result;
}

void inspect_packets(std::istream & input, std::ostream & output)
{
  std::uint64_t index = 0;
  while (true) {
    std::array<std::uint8_t, 4> prefix{};
    input.read(reinterpret_cast<char *>(prefix.data()), prefix.size());
    if (input.gcount() == 0 && input.eof()) {return;}
    if (input.gcount() != 4) {throw std::runtime_error("Incomplete timing request length");}
    const auto size = static_cast<std::uint32_t>(prefix[0]) |
      (static_cast<std::uint32_t>(prefix[1]) << 8) |
      (static_cast<std::uint32_t>(prefix[2]) << 16) |
      (static_cast<std::uint32_t>(prefix[3]) << 24);
    if (size > protocol::kMaximumUdpPacketSize) {
      throw std::runtime_error("Oversized timing request");
    }
    std::vector<std::uint8_t> bytes(size);
    input.read(reinterpret_cast<char *>(bytes.data()), size);
    if (static_cast<std::uint32_t>(input.gcount()) != size) {
      throw std::runtime_error("Incomplete timing request payload");
    }
    auto result = describe_packet(bytes);
    result["input_index"] = index++;
    output << result.dump() << std::endl;
    if (!output) {throw std::runtime_error("Could not write timing response");}
  }
}
}  // namespace sonar3d::timing
