// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "sonar3d/protocol.hpp"

#include <google/protobuf/any.pb.h>
#include <snappy.h>
#include <zlib.h>

#include <array>
#include <cstring>
#include <limits>
#include <sstream>
#include <string_view>
#include <type_traits>
#include <utility>

#include "WaterLinkedSonarIntegrationProtocol.pb.h"

namespace sonar3d::protocol
{
namespace
{

namespace vendor = waterlinked::sonar::protocol;

constexpr std::size_t kIdentifierSize = 4;
constexpr std::size_t kLengthSize = 4;
constexpr std::size_t kChecksumSize = 4;
constexpr std::size_t kHeaderSize = kIdentifierSize + kLengthSize;
constexpr std::size_t kMinimumPacketSize = kHeaderSize + kChecksumSize;
constexpr std::string_view kRip1Identifier = "RIP1";
constexpr std::string_view kRip2Identifier = "RIP2";

[[nodiscard]] std::uint32_t read_little_endian_u32(std::span<const std::uint8_t, 4> bytes)
{
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

void append_little_endian_u32(std::vector<std::uint8_t> & bytes, std::uint32_t value)
{
  bytes.push_back(static_cast<std::uint8_t>(value & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xffU));
  bytes.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xffU));
}

[[nodiscard]] std::uint32_t packet_crc(std::span<const std::uint8_t> bytes)
{
  auto crc = crc32(0L, Z_NULL, 0);
  crc = crc32(crc, reinterpret_cast<const Bytef *>(bytes.data()), static_cast<uInt>(bytes.size()));
  return static_cast<std::uint32_t>(crc);
}

[[nodiscard]] ProtocolVersion protocol_version(std::span<const std::uint8_t> identifier)
{
  const std::string_view value(reinterpret_cast<const char *>(identifier.data()),
    identifier.size());
  if (value == kRip1Identifier) {
    return ProtocolVersion::RIP1;
  }
  if (value == kRip2Identifier) {
    return ProtocolVersion::RIP2;
  }

  std::ostringstream message;
  message << "invalid RIP packet identifier";
  throw ProtocolError(ErrorCode::BAD_IDENTIFIER, false, message.str());
}

[[nodiscard]] MessageHeader from_vendor_header(const vendor::Header & source)
{
  MessageHeader result;
  result.sequence_id = source.sequence_id();
  if (source.has_timestamp()) {
    result.timestamp = Timestamp{
      source.timestamp().seconds(),
      source.timestamp().nanos(),
    };
  }
  return result;
}

void to_vendor_header(const MessageHeader & source, vendor::Header * destination)
{
  destination->set_sequence_id(source.sequence_id);
  if (source.timestamp) {
    destination->mutable_timestamp()->set_seconds(source.timestamp->seconds);
    destination->mutable_timestamp()->set_nanos(source.timestamp->nanoseconds);
  }
}

[[nodiscard]] RangeImage from_vendor_range_image(const vendor::RangeImage & source)
{
  RangeImage result;
  if (source.has_header()) {
    result.header = from_vendor_header(source.header());
  }
  result.speed_of_sound = source.speed_of_sound();
  result.range = source.range();
  result.frequency = source.frequency();
  result.width = source.width();
  result.height = source.height();
  result.horizontal_fov_degrees = source.fov_horizontal();
  result.vertical_fov_degrees = source.fov_vertical();
  result.pixel_scale = source.image_pixel_scale();
  result.pixels.assign(source.image_pixel_data().begin(), source.image_pixel_data().end());
  return result;
}

[[nodiscard]] BitmapImage from_vendor_bitmap_image(const vendor::BitmapImageGreyscale8 & source)
{
  BitmapImage result;
  if (source.has_header()) {
    result.header = from_vendor_header(source.header());
  }
  result.speed_of_sound = source.speed_of_sound();
  result.range = source.range();
  result.frequency = source.frequency();
  result.type = static_cast<BitmapImageType>(source.type());
  result.width = source.width();
  result.height = source.height();
  result.horizontal_fov_degrees = source.fov_horizontal();
  result.vertical_fov_degrees = source.fov_vertical();
  const auto & pixels = source.image_pixel_data();
  result.pixels.assign(pixels.begin(), pixels.end());
  return result;
}

[[nodiscard]] ImuBatch from_vendor_imu_batch(const vendor::ImuBatch & source)
{
  ImuBatch result;
  result.sequence_id = source.batch_sequence_id();
  result.sample_count = source.samples();
  result.timestamps.reserve(static_cast<std::size_t>(source.timestamp_size()));
  for (const auto & timestamp : source.timestamp()) {
    result.timestamps.push_back(Timestamp{timestamp.seconds(), timestamp.nanos()});
  }
  result.specific_force.assign(source.specific_force().begin(), source.specific_force().end());
  result.rate_of_turn.assign(source.rate_of_turn().begin(), source.rate_of_turn().end());
  return result;
}

void pack_message(const RangeImage & source, google::protobuf::Any * destination)
{
  vendor::RangeImage message;
  to_vendor_header(source.header, message.mutable_header());
  message.set_speed_of_sound(source.speed_of_sound);
  message.set_range(source.range);
  message.set_frequency(source.frequency);
  message.set_width(source.width);
  message.set_height(source.height);
  message.set_fov_horizontal(source.horizontal_fov_degrees);
  message.set_fov_vertical(source.vertical_fov_degrees);
  message.set_image_pixel_scale(source.pixel_scale);
  for (const auto pixel : source.pixels) {
    message.add_image_pixel_data(pixel);
  }
  destination->PackFrom(message);
}

void pack_message(const BitmapImage & source, google::protobuf::Any * destination)
{
  vendor::BitmapImageGreyscale8 message;
  to_vendor_header(source.header, message.mutable_header());
  message.set_speed_of_sound(source.speed_of_sound);
  message.set_range(source.range);
  message.set_frequency(source.frequency);
  message.set_type(static_cast<vendor::BitmapImageType>(source.type));
  message.set_width(source.width);
  message.set_height(source.height);
  message.set_fov_horizontal(source.horizontal_fov_degrees);
  message.set_fov_vertical(source.vertical_fov_degrees);
  message.set_image_pixel_data(source.pixels.data(), source.pixels.size());
  destination->PackFrom(message);
}

void pack_message(const ImuBatch & source, google::protobuf::Any * destination)
{
  vendor::ImuBatch message;
  message.set_batch_sequence_id(source.sequence_id);
  message.set_samples(source.sample_count);
  for (const auto & timestamp : source.timestamps) {
    auto * output = message.add_timestamp();
    output->set_seconds(timestamp.seconds);
    output->set_nanos(timestamp.nanoseconds);
  }
  for (const auto value : source.specific_force) {
    message.add_specific_force(value);
  }
  for (const auto value : source.rate_of_turn) {
    message.add_rate_of_turn(value);
  }
  destination->PackFrom(message);
}

void pack_message(const UnknownMessage &, google::protobuf::Any *)
{
  throw std::invalid_argument("cannot encode an unknown protobuf message without its payload");
}

[[nodiscard]] std::string serialize_message(const Message & source)
{
  vendor::Packet packet;
  std::visit(
    [&packet](const auto & value) {
      pack_message(value, packet.mutable_msg());
    },
    source);

  std::string payload;
  if (!packet.SerializeToString(&payload)) {
    throw std::runtime_error("failed to serialize RIP protobuf packet");
  }
  return payload;
}

}  // namespace

ProtocolError::ProtocolError(ErrorCode code, bool framing_intact, std::string message)
: std::runtime_error(std::move(message)), code_(code), framing_intact_(framing_intact)
{
}

ErrorCode ProtocolError::code() const noexcept
{
  return code_;
}

bool ProtocolError::framing_intact() const noexcept
{
  return framing_intact_;
}

DecodedPacket decode_packet(std::span<const std::uint8_t> bytes)
{
  if (bytes.size() < kHeaderSize) {
    throw ProtocolError(ErrorCode::INCOMPLETE_PACKET, false, "incomplete RIP packet header");
  }

  const auto version = protocol_version(bytes.first(kIdentifierSize));
  const auto declared_size = read_little_endian_u32(
    std::span<const std::uint8_t, 4>(bytes.data() + kIdentifierSize, kLengthSize));

  if (declared_size < kMinimumPacketSize) {
    throw ProtocolError(
      ErrorCode::INVALID_LENGTH,
      false,
      "RIP packet length is smaller than the framing overhead");
  }
  if (declared_size > kMaximumUdpPacketSize) {
    throw ProtocolError(
      ErrorCode::INVALID_LENGTH,
      false,
      "RIP packet length exceeds the maximum UDP datagram size");
  }
  if (declared_size > bytes.size()) {
    throw ProtocolError(ErrorCode::INCOMPLETE_PACKET, false, "incomplete RIP packet payload");
  }
  if (declared_size < bytes.size()) {
    throw ProtocolError(ErrorCode::EXTRA_DATA, false, "extra bytes follow the RIP packet");
  }

  const auto expected_crc = packet_crc(bytes.first(bytes.size() - kChecksumSize));
  const auto received_crc = read_little_endian_u32(
    std::span<const std::uint8_t, 4>(bytes.data() + bytes.size() - kChecksumSize, kChecksumSize));
  if (expected_crc != received_crc) {
    std::ostringstream message;
    message << "RIP packet CRC mismatch: expected 0x" << std::hex << expected_crc << ", received 0x"
            << received_crc;
    throw ProtocolError(ErrorCode::CRC_MISMATCH, true, message.str());
  }

  const auto encoded_payload = bytes.subspan(kHeaderSize, bytes.size() - kMinimumPacketSize);
  std::string decoded_payload;
  if (version == ProtocolVersion::RIP1) {
    decoded_payload.assign(
      reinterpret_cast<const char *>(encoded_payload.data()),
      encoded_payload.size());
  } else {
    std::size_t decoded_size{};
    if (!snappy::GetUncompressedLength(
        reinterpret_cast<const char *>(encoded_payload.data()), encoded_payload.size(),
        &decoded_size))
    {
      throw ProtocolError(ErrorCode::DECOMPRESSION_FAILED, true, "invalid RIP2 Snappy payload");
    }
    if (decoded_size > kMaximumDecodedPayloadSize) {
      throw ProtocolError(
        ErrorCode::PAYLOAD_TOO_LARGE,
        true,
        "decoded RIP2 payload exceeds the configured safety limit");
    }
    decoded_payload.resize(decoded_size);
    if (!snappy::RawUncompress(
        reinterpret_cast<const char *>(encoded_payload.data()),
        encoded_payload.size(),
        decoded_payload.data()))
    {
      throw ProtocolError(ErrorCode::DECOMPRESSION_FAILED, true,
          "failed to decompress RIP2 payload");
    }
  }

  if (decoded_payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw ProtocolError(ErrorCode::PAYLOAD_TOO_LARGE, true,
        "protobuf payload cannot be represented safely");
  }

  vendor::Packet packet;
  if (!packet.ParseFromArray(decoded_payload.data(), static_cast<int>(decoded_payload.size())) ||
    !packet.has_msg())
  {
    throw ProtocolError(ErrorCode::PROTOBUF_PARSE_FAILED, true,
        "failed to parse RIP protobuf payload");
  }

  if (packet.msg().Is<vendor::RangeImage>()) {
    vendor::RangeImage message;
    if (!packet.msg().UnpackTo(&message)) {
      throw ProtocolError(ErrorCode::PROTOBUF_PARSE_FAILED, true, "failed to unpack RangeImage");
    }
    return DecodedPacket{version, from_vendor_range_image(message)};
  }
  if (packet.msg().Is<vendor::BitmapImageGreyscale8>()) {
    vendor::BitmapImageGreyscale8 message;
    if (!packet.msg().UnpackTo(&message)) {
      throw ProtocolError(
        ErrorCode::PROTOBUF_PARSE_FAILED,
        true,
        "failed to unpack BitmapImageGreyscale8");
    }
    return DecodedPacket{version, from_vendor_bitmap_image(message)};
  }
  if (packet.msg().Is<vendor::ImuBatch>()) {
    vendor::ImuBatch message;
    if (!packet.msg().UnpackTo(&message)) {
      throw ProtocolError(ErrorCode::PROTOBUF_PARSE_FAILED, true, "failed to unpack ImuBatch");
    }
    return DecodedPacket{version, from_vendor_imu_batch(message)};
  }

  return DecodedPacket{version, UnknownMessage{packet.msg().type_url()}};
}

std::optional<DecodedPacket> read_packet(std::istream & stream)
{
  std::array<std::uint8_t, kHeaderSize> header{};
  stream.read(reinterpret_cast<char *>(header.data()), static_cast<std::streamsize>(header.size()));
  const auto header_bytes = static_cast<std::size_t>(stream.gcount());
  if (header_bytes == 0 && stream.eof()) {
    return std::nullopt;
  }
  if (header_bytes != header.size()) {
    throw ProtocolError(ErrorCode::INCOMPLETE_PACKET, false,
        "incomplete RIP packet header in recording");
  }

  static_cast<void>(protocol_version(std::span<const std::uint8_t>(header).first(kIdentifierSize)));
  const auto declared_size = read_little_endian_u32(
    std::span<const std::uint8_t, 4>(header.data() + kIdentifierSize, kLengthSize));
  if (declared_size < kMinimumPacketSize || declared_size > kMaximumUdpPacketSize) {
    throw ProtocolError(ErrorCode::INVALID_LENGTH, false, "invalid RIP packet length in recording");
  }

  std::vector<std::uint8_t> packet(declared_size);
  std::memcpy(packet.data(), header.data(), header.size());
  const auto remaining_size = packet.size() - header.size();
  stream.read(
    reinterpret_cast<char *>(packet.data() + header.size()),
    static_cast<std::streamsize>(remaining_size));
  if (static_cast<std::size_t>(stream.gcount()) != remaining_size) {
    throw ProtocolError(ErrorCode::INCOMPLETE_PACKET, false, "incomplete RIP packet in recording");
  }

  return decode_packet(packet);
}

std::vector<std::uint8_t> encode_packet(const Message & message, ProtocolVersion version)
{
  const auto protobuf_payload = serialize_message(message);
  std::string encoded_payload;
  std::string_view identifier;
  switch (version) {
    case ProtocolVersion::RIP1:
      encoded_payload = protobuf_payload;
      identifier = kRip1Identifier;
      break;
    case ProtocolVersion::RIP2:
      snappy::Compress(protobuf_payload.data(), protobuf_payload.size(), &encoded_payload);
      identifier = kRip2Identifier;
      break;
    default:
      throw std::invalid_argument("unsupported RIP protocol version");
  }

  const auto packet_size = kMinimumPacketSize + encoded_payload.size();
  if (packet_size > kMaximumUdpPacketSize ||
    packet_size > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::length_error("encoded RIP packet exceeds the maximum UDP datagram size");
  }

  std::vector<std::uint8_t> result;
  result.reserve(packet_size);
  result.insert(result.end(), identifier.begin(), identifier.end());
  append_little_endian_u32(result, static_cast<std::uint32_t>(packet_size));
  result.insert(result.end(), encoded_payload.begin(), encoded_payload.end());
  append_little_endian_u32(result, packet_crc(result));
  return result;
}

}  // namespace sonar3d::protocol
