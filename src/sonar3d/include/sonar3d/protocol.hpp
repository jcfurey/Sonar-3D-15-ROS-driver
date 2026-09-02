// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <span>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace sonar3d::protocol
{

inline constexpr std::size_t kMaximumUdpPacketSize = 65'507;
inline constexpr std::size_t kMaximumDecodedPayloadSize = 4U * 1024U * 1024U;

enum class ProtocolVersion : std::uint8_t
{
  RIP1 = 1,
  RIP2 = 2,
};

struct Timestamp
{
  std::int64_t seconds{};
  std::int32_t nanoseconds{};

  bool operator==(const Timestamp &) const = default;
};

struct MessageHeader
{
  std::optional<Timestamp> timestamp;
  std::uint32_t sequence_id{};

  bool operator==(const MessageHeader &) const = default;
};

enum class BitmapImageType : std::int32_t
{
  SIGNAL_STRENGTH_IMAGE = 0,
  SHADED_IMAGE = 1,
};

struct BitmapImage
{
  MessageHeader header;
  float speed_of_sound{};
  float range{};
  std::uint32_t frequency{};
  BitmapImageType type{BitmapImageType::SIGNAL_STRENGTH_IMAGE};
  std::uint32_t width{};
  std::uint32_t height{};
  float horizontal_fov_degrees{};
  float vertical_fov_degrees{};
  std::vector<std::uint8_t> pixels;

  bool operator==(const BitmapImage &) const = default;
};

struct RangeImage
{
  MessageHeader header;
  float speed_of_sound{};
  float range{};
  std::uint32_t frequency{};
  std::uint32_t width{};
  std::uint32_t height{};
  float horizontal_fov_degrees{};
  float vertical_fov_degrees{};
  float pixel_scale{};
  std::vector<std::uint32_t> pixels;

  bool operator==(const RangeImage &) const = default;
};

struct ImuBatch
{
  std::uint32_t sequence_id{};
  std::uint32_t sample_count{};
  std::vector<Timestamp> timestamps;
  std::vector<float> specific_force;
  std::vector<float> rate_of_turn;

  bool operator==(const ImuBatch &) const = default;
};

struct UnknownMessage
{
  std::string type_url;

  bool operator==(const UnknownMessage &) const = default;
};

using Message = std::variant<RangeImage, BitmapImage, ImuBatch, UnknownMessage>;

struct DecodedPacket
{
  ProtocolVersion version;
  Message message;
};

enum class ErrorCode
{
  BAD_IDENTIFIER,
  INVALID_LENGTH,
  INCOMPLETE_PACKET,
  EXTRA_DATA,
  CRC_MISMATCH,
  DECOMPRESSION_FAILED,
  PAYLOAD_TOO_LARGE,
  PROTOBUF_PARSE_FAILED,
};

class ProtocolError : public std::runtime_error
{
public:
  ProtocolError(ErrorCode code, bool framing_intact, std::string message);

  [[nodiscard]] ErrorCode code() const noexcept;
  [[nodiscard]] bool framing_intact() const noexcept;

private:
  ErrorCode code_;
  bool framing_intact_;
};

[[nodiscard]] DecodedPacket decode_packet(std::span<const std::uint8_t> bytes);

// Read and decode one length-framed packet. A clean EOF returns nullopt. Once a
// complete, sane-length packet has been consumed, errors report framing_intact
// so recording playback can skip that packet and continue safely.
[[nodiscard]] std::optional<DecodedPacket> read_packet(std::istream & stream);

// Primarily intended for protocol regression tests and diagnostic tooling.
[[nodiscard]] std::vector<std::uint8_t> encode_packet(
  const Message & message,
  ProtocolVersion version = ProtocolVersion::RIP2);

}  // namespace sonar3d::protocol
