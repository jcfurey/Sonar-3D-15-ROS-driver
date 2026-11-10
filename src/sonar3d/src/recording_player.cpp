// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/default_storage_id.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/playback_clock.hpp"
#include "sonar3d/protocol.hpp"
#include "sonar3d/publisher_demand.hpp"
#include "sonar3d/range_signal_pairing.hpp"
#include "sonar3d/msg/raw_packet.hpp"

namespace sonar3d
{
namespace
{

struct PlaybackOptions
{
  std::string file;
  std::string frame_id{"sonar3d_link"};
  std::string imu_frame_id{"sonar3d_imu_link"};
  std::string output;
  std::string storage_id;
  double realtime_factor{1.0};
  double startup_delay{0.5};
  bool use_sensor_timestamps{true};
  bool intensity{true};
  bool validate_only{false};
  bool vendor_orientation{false};
  std::string orientation_frame;
  bool help{false};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage: " << executable << " --file RECORDING [options]\n"
    << "\n"
    << "Publish a mixed RIP1/RIP2 .sonar recording using the live driver's topics,\n"
    << "or write those topics straight to a rosbag2 bag with --output.\n"
    << "\n"
    << "Options:\n"
    << "  --file PATH                 Recording to play (required)\n"
    << "  --output BAG               Write every product to a new bag instead of publishing\n"
    << "  --storage ID               Bag storage plugin, e.g. mcap or sqlite3 (default: mcap)\n"
    << "  --realtime-factor FACTOR   Playback speed multiplier (default: 1.0)\n"
    << "  --frame-id FRAME           Range/image/cloud frame (default: sonar3d_link)\n"
    << "  --imu-frame-id FRAME       IMU frame (default: sonar3d_imu_link)\n"
    << "  --startup-delay SECONDS    DDS discovery delay (default: 0.5)\n"
    << "  --receive-time             Stamp products from the replay ROS clock\n"
    << "  --no-intensity             Omit the point cloud intensity field\n"
    << "  --validate-only            Check all supported conversions without publishing or pacing\n"
    << "  --vendor-orientation       Publish experimental private attitude on imu/data\n"
    << "  --orientation-frame FRAME  Attitude-only preview TF parent "
    "(requires --vendor-orientation)\n"
    << "  -h, --help                 Show this help\n";
}

[[nodiscard]] std::string option_value(
  const std::vector<std::string> & arguments,
  std::size_t & index,
  const std::string & option)
{
  const auto prefix = option + "=";
  if (arguments[index].rfind(prefix, 0) == 0) {
    return arguments[index].substr(prefix.size());
  }
  if (index + 1 >= arguments.size()) {
    throw std::invalid_argument(option + " requires a value");
  }
  ++index;
  return arguments[index];
}

[[nodiscard]] double parse_number(const std::string & text, const std::string & option)
{
  std::size_t parsed{};
  const auto value = std::stod(text, &parsed);
  if (parsed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(option + " must be a finite number");
  }
  return value;
}

[[nodiscard]] PlaybackOptions parse_arguments(const std::vector<std::string> & arguments)
{
  PlaybackOptions options;
  for (std::size_t index = 1; index < arguments.size(); ++index) {
    const auto & argument = arguments[index];
    if (argument == "-h" || argument == "--help") {
      options.help = true;
    } else if (argument == "--receive-time") {
      options.use_sensor_timestamps = false;
    } else if (argument == "--no-intensity") {
      options.intensity = false;
    } else if (argument == "--validate-only") {
      options.validate_only = true;
    } else if (argument == "--vendor-orientation") {
      options.vendor_orientation = true;
    } else if (argument == "--orientation-frame" || argument.starts_with("--orientation-frame=")) {
      options.orientation_frame = option_value(arguments, index, "--orientation-frame");
    } else if (argument == "--file" || argument.rfind("--file=", 0) == 0) {
      options.file = option_value(arguments, index, "--file");
    } else if (argument == "--output" || argument.rfind("--output=", 0) == 0) {
      options.output = option_value(arguments, index, "--output");
    } else if (argument == "--storage" || argument.rfind("--storage=", 0) == 0) {
      options.storage_id = option_value(arguments, index, "--storage");
    } else if (argument == "--frame-id" || argument.rfind("--frame-id=", 0) == 0) {
      options.frame_id = option_value(arguments, index, "--frame-id");
    } else if (argument == "--imu-frame-id" || argument.rfind("--imu-frame-id=", 0) == 0) {
      options.imu_frame_id = option_value(arguments, index, "--imu-frame-id");
    } else if (argument == "--realtime-factor" || argument.rfind("--realtime-factor=", 0) == 0) {
      options.realtime_factor = parse_number(
        option_value(arguments, index, "--realtime-factor"), "--realtime-factor");
    } else if (argument == "--startup-delay" || argument.rfind("--startup-delay=", 0) == 0) {
      options.startup_delay = parse_number(
        option_value(arguments, index, "--startup-delay"), "--startup-delay");
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }

  if (!options.help && options.file.empty()) {
    throw std::invalid_argument("--file is required");
  }
  if (options.realtime_factor <= 0.0) {
    throw std::invalid_argument("--realtime-factor must be greater than zero");
  }
  if (options.startup_delay < 0.0) {
    throw std::invalid_argument("--startup-delay must not be negative");
  }
  if (options.frame_id.empty() || options.imu_frame_id.empty()) {
    throw std::invalid_argument("frame IDs must not be empty");
  }
  if (!options.output.empty() && options.validate_only) {
    throw std::invalid_argument("--output and --validate-only are mutually exclusive");
  }
  if (!options.output.empty() && !options.use_sensor_timestamps) {
    throw std::invalid_argument(
            "--receive-time cannot be used with --output; bags keep recorded sensor timestamps");
  }
  if (options.output.empty() && !options.storage_id.empty()) {
    throw std::invalid_argument("--storage requires --output");
  }
  if (!options.orientation_frame.empty() &&
    (!options.vendor_orientation || !options.output.empty() || options.validate_only ||
    options.orientation_frame == options.frame_id ||
    options.orientation_frame == options.imu_frame_id))
  {
    throw std::invalid_argument(
      "--orientation-frame requires --vendor-orientation, publishing mode, "
      "and a distinct parent frame");
  }
  return options;
}

[[nodiscard]] std::optional<std::int64_t> sensor_time_nanoseconds(const protocol::Message & message)
{
  const auto to_nanoseconds = [](const protocol::Timestamp & stamp) -> std::optional<std::int64_t> {
      if (stamp.seconds == 0 && stamp.nanoseconds == 0) {
        return std::nullopt;
      }
      if (stamp.nanoseconds < 0 || stamp.nanoseconds >= 1'000'000'000 ||
        stamp.seconds < std::numeric_limits<std::int32_t>::min() ||
        stamp.seconds > std::numeric_limits<std::int32_t>::max())
      {
        return std::nullopt;
      }
      return stamp.seconds * 1'000'000'000 + stamp.nanoseconds;
    };

  return std::visit(
    [&to_nanoseconds](const auto & value) -> std::optional<std::int64_t> {
      using MessageType = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<MessageType, protocol::RangeImage>||
      std::is_same_v<MessageType, protocol::BitmapImage>||
      std::is_same_v<MessageType, protocol::ImuOrientation>)
      {
        return value.header.timestamp ? to_nanoseconds(*value.header.timestamp) : std::nullopt;
      } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
        return value.timestamps.empty() ? std::nullopt : to_nanoseconds(value.timestamps.front());
      } else {
        return std::nullopt;
      }
    },
    message);
}

void interruptible_sleep_until(std::chrono::steady_clock::time_point deadline)
{
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const std::chrono::duration<double> remaining = deadline - std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::min(remaining, std::chrono::duration<double>(0.05)));
  }
}

void interruptible_sleep(double seconds)
{
  interruptible_sleep_until(std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds)));
}

[[nodiscard]] std::optional<std::string> unsupported_type(const protocol::Message & message)
{
  if (const auto * unknown = std::get_if<protocol::UnknownMessage>(&message)) {
    return unknown->type_url;
  }
  if (const auto * bitmap = std::get_if<protocol::BitmapImage>(&message)) {
    if (bitmap->type != protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE &&
      bitmap->type != protocol::BitmapImageType::SHADED_IMAGE)
    {
      return "waterlinked.sonar.protocol.BitmapImageGreyscale8/type=" +
             std::to_string(static_cast<int>(bitmap->type));
    }
  }
  return std::nullopt;
}

enum class OutputMode
{
  PUBLISH,
  VALIDATE,
  BAG,
};

// Validate before pacing or updating the cloud pairing state, including when
// nothing subscribes to a product. Invalid timestamps must never move the
// playback clock's anchor and delay later valid packets.
void validate_recorded_message(const protocol::Message & source, bool use_sensor_timestamps)
{
  std::visit(
    [use_sensor_timestamps](const auto & message) {
      using MessageType = std::decay_t<decltype(message)>;
      if constexpr (std::is_same_v<MessageType, protocol::RangeImage>) {
        conversions::validate_point_cloud_source(message);
        if (use_sensor_timestamps) {
          conversions::validate_sensor_timestamp(message.header.timestamp);
        }
      } else if constexpr (std::is_same_v<MessageType, protocol::BitmapImage>) {
        conversions::validate_bitmap_source(message);
        if (use_sensor_timestamps) {
          conversions::validate_sensor_timestamp(message.header.timestamp);
        }
      } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
        conversions::validate_imu_source(message, use_sensor_timestamps);
      } else if constexpr (std::is_same_v<MessageType, protocol::ImuOrientation>) {
        conversions::validate_orientation_source(message, use_sensor_timestamps);
      }
    }, source);
}

class RecordingPlayer final : public rclcpp::Node
{
public:
  explicit RecordingPlayer(const PlaybackOptions & options)
  : Node("sonar3d_playback"),
    frame_id_(options.frame_id),
    imu_frame_id_(options.imu_frame_id),
    use_sensor_timestamps_(options.use_sensor_timestamps),
    intensity_(options.intensity),
    vendor_orientation_(options.vendor_orientation),
    orientation_frame_(options.orientation_frame),
    mode_(options.validate_only ? OutputMode::VALIDATE :
      (options.output.empty() ? OutputMode::PUBLISH : OutputMode::BAG))
  {
    range_image_.topic = resolve("range_image");
    intensity_image_.topic = resolve("intensity_image");
    shaded_image_.topic = resolve("shaded_image");
    point_cloud_.topic = resolve("points");
    imu_.topic = resolve("imu/data_raw");
    orientation_.topic = resolve("imu/data");
    image_metadata_.topic = resolve("image_metadata");
    raw_packet_.topic = resolve("raw_packet");
    if (mode_ == OutputMode::BAG) {
      rosbag2_storage::StorageOptions storage;
      storage.uri = options.output;
      storage.storage_id = options.storage_id.empty() ?
        rosbag2_storage::get_default_storage_id() : options.storage_id;
      writer_ = std::make_unique<rosbag2_cpp::Writer>();
      writer_->open(storage);
    } else if (mode_ == OutputMode::PUBLISH) {
      // Reliable, as REP-2003 asks of sensor sources; the deep history covers
      // IMU batches published back to back.
      const auto qos = rclcpp::SystemDefaultsQoS().keep_last(100);
      advertise(range_image_, qos);
      advertise(intensity_image_, qos);
      advertise(shaded_image_, qos);
      advertise(point_cloud_, qos);
      advertise(imu_, qos);
      if (vendor_orientation_) {
        advertise(orientation_, qos);
      }
      advertise(image_metadata_, qos);
      advertise(raw_packet_, qos);
      if (!orientation_frame_.empty()) {
        orientation_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_WARN(get_logger(),
          "Experimental attitude-only preview: local z-up reference, "
          "unverified heading, no translation");
      }
    }
  }

  [[nodiscard]] bool wants_raw_packets() const
  {
    return mode_ != OutputMode::VALIDATE && wanted(raw_packet_);
  }

  void recording_origin(std::optional<std::int64_t> stamp) {raw_stamp_ = stamp;}

  void process_packet(
    std::vector<std::uint8_t> && bytes, std::optional<std::int64_t> stamp = std::nullopt)
  {
    const auto index = raw_packet_index_++;
    if (stamp && *stamp >= 0) {
      raw_stamp_ = stamp;
    }
    if (!wants_raw_packets() || bytes.empty()) {
      return;
    }
    msg::RawPacket raw;
    raw.header.frame_id = frame_id_;
    raw.packet_index = index;
    raw.header.stamp = use_sensor_timestamps_ ? rclcpp::Time(raw_stamp_.value_or(0)) : now();
    raw.data = std::move(bytes);
    emit(raw_packet_, std::move(raw));
  }

  void process_message(protocol::Message && source)
  {
    observe_recording_epoch(source);
    const builtin_interfaces::msg::Time fallback_stamp = now();
    std::visit(
      [this, &fallback_stamp](auto & message) {
        using MessageType = std::decay_t<decltype(message)>;
        if constexpr (std::is_same_v<MessageType, protocol::RangeImage>) {
          const auto header = conversions::make_header(
            message.header,
            frame_id_,
            fallback_stamp,
            use_sensor_timestamps_);
          // Validate everything before emitting, so a malformed image never
          // produces only part of its outputs.
          std::optional<sensor_msgs::msg::Image> image;
          if (wanted(range_image_)) {
            image = conversions::make_range_image(message, header);
          }
          bool cloud = wanted(point_cloud_);
          if (orientation_tf_) {
            if (last_orientation_ && last_orientation_->header == message.header) {
              const auto imu = conversions::make_orientation_message(*last_orientation_,
                imu_frame_id_, fallback_stamp, use_sensor_timestamps_);
              geometry_msgs::msg::TransformStamped transform;
              transform.header = header;
              transform.header.frame_id = orientation_frame_;
              transform.child_frame_id = frame_id_;
              transform.transform.rotation = imu.orientation;
              orientation_tf_->sendTransform(transform);
            } else {
              // Keep the raw image, but never render a cloud using another
              // shot's attitude (including a missing first orientation).
              cloud = false;
              ++missing_orientations_;
            }
          }
          if (cloud) {
            conversions::validate_point_cloud_source(message);
          }
          std::optional<msg::ImageMetadata> metadata;
          if (wanted(image_metadata_)) {
            metadata = conversions::make_image_metadata(message, header);
          }
          if (image) {
            emit(range_image_, std::move(*image));
          }
          if (metadata) {
            emit(image_metadata_, std::move(*metadata));
          }
          if (cloud && intensity_) {
            // The cloud waits for this shot's signal-strength image.
            for (auto & pair : pairing_.add_range(std::move(message),
                CloudContext{header, std::chrono::steady_clock::now() +
              std::chrono::milliseconds(25)}))
            {
              emit_cloud(std::move(pair));
            }
          } else if (cloud) {
            emit(point_cloud_, conversions::make_point_cloud(message, header));
          }
        } else if constexpr (std::is_same_v<MessageType, protocol::BitmapImage>) {
          auto & output = message.type == protocol::BitmapImageType::SHADED_IMAGE ?
          shaded_image_ : intensity_image_;
          if (wanted(output) || wanted(image_metadata_)) {
            const auto header = conversions::make_header(
              message.header, frame_id_, fallback_stamp, use_sensor_timestamps_);
            if (wanted(output)) {
              emit(output, conversions::make_bitmap_image(message, header));
            }
            if (wanted(image_metadata_)) {
              emit(image_metadata_, conversions::make_image_metadata(message, header));
            }
          }
          if (message.type == protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE && intensity_ &&
          wanted(point_cloud_))
          {
            for (auto & pair : pairing_.add_signal(std::move(message))) {
              emit_cloud(std::move(pair));
            }
          }
        } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
          if (wanted(imu_)) {
            auto messages = conversions::make_imu_messages(
              message, imu_frame_id_, fallback_stamp, use_sensor_timestamps_);
            for (auto & imu : messages) {
              emit(imu_, std::move(imu));
            }
          }
        } else if constexpr (std::is_same_v<MessageType, protocol::ImuOrientation>) {
          if ((vendor_orientation_ && wanted(orientation_)) || mode_ == OutputMode::VALIDATE) {
            emit(orientation_, conversions::make_orientation_message(message,
              imu_frame_id_, fallback_stamp, use_sensor_timestamps_));
          }
          if (orientation_tf_) {
            last_orientation_ = message;
          }
        }
      },
      source);
  }

  // Emit a cloud still waiting for a signal-strength image at end of input.
  void flush()
  {
    if (auto pair = pairing_.release()) {
      emit_cloud(std::move(*pair));
    }
  }

  // Service the live driver's 25 ms intensity deadline while the file player
  // waits for the next recorded packet. Offline bag conversion keeps pairing
  // by recorded shot without imposing wall-clock time on the conversion.
  void wait_until(
    std::chrono::steady_clock::time_point deadline,
    rclcpp::executors::SingleThreadedExecutor & executor)
  {
    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      const auto * pending = pairing_.pending_context();
      if (pending && now >= pending->deadline) {
        flush();
        continue;
      }
      if (now >= deadline) {
        return;
      }
      auto wake = std::min(deadline, now + std::chrono::milliseconds(50));
      if (pending) {
        wake = std::min(wake, pending->deadline);
      }
      std::this_thread::sleep_until(wake);
      executor.spin_some();
    }
  }

  [[nodiscard]] std::uint64_t unpaired_range_images() const{return unpaired_range_images_;}
  [[nodiscard]] std::uint64_t missing_orientations() const{return missing_orientations_;}

  [[nodiscard]] const std::map<std::string, std::uint64_t> & outputs() const{return outputs_;}
  [[nodiscard]] OutputMode mode() const{return mode_;}

  // Close the bag so its metadata is complete before the summary is printed.
  void finish() {writer_.reset();}

private:
  void observe_recording_epoch(const protocol::Message & source)
  {
    const auto * range = std::get_if<protocol::RangeImage>(&source);
    const auto * bitmap = std::get_if<protocol::BitmapImage>(&source);
    if (!range && (!bitmap || bitmap->type != protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE)) {
      return;
    }
    const auto & header = range ? range->header : bitmap->header;
    auto & previous = range ? last_range_header_ : last_signal_header_;
    const auto stamp = conversions::sensor_nanoseconds(header.timestamp);
    const auto previous_stamp = previous ?
      conversions::sensor_nanoseconds(previous->timestamp) : std::nullopt;
    bool restarted = stamp && latest_image_stamp_ && *latest_image_stamp_ - *stamp > 1'000'000'000;
    if (previous) {
      const auto delta = header.sequence_id - previous->sequence_id;
      // A repeated/backward counter with newer acquisition time is a new
      // recording segment. A late packet with an older stamp is just reordered.
      restarted |= (delta == 0 || delta >= (std::uint32_t{1} << 31U)) &&
        stamp && previous_stamp && *stamp > *previous_stamp;
    }
    if (restarted) {
      flush();
      pairing_.clear();
      last_range_header_.reset();
      last_signal_header_.reset();
      latest_image_stamp_.reset();
    }
    previous = header;
    if (stamp) {
      latest_image_stamp_ = latest_image_stamp_ ? std::max(*latest_image_stamp_, *stamp) : *stamp;
    }
  }

  struct CloudContext
  {
    std_msgs::msg::Header header;
    std::chrono::steady_clock::time_point deadline;
  };
  using CloudPairing = RangeSignalPairing<CloudContext>;

  template<typename MessageT>
  struct Output
  {
    std::string topic;
    typename rclcpp::Publisher<MessageT>::SharedPtr publisher;
  };

  [[nodiscard]] std::string resolve(const std::string & topic)
  {
    return get_node_topics_interface()->resolve_topic_name(topic);
  }

  template<typename MessageT>
  void advertise(Output<MessageT> & output, const rclcpp::QoS & qos)
  {
    output.publisher = create_publisher<MessageT>(output.topic, qos);
  }

  // Playback converts only products with subscribers; validation and bag
  // conversion handle every product.
  template<typename MessageT>
  [[nodiscard]] bool wanted(const Output<MessageT> & output) const
  {
    return mode_ != OutputMode::PUBLISH || has_subscribers(output.publisher);
  }

  void emit_cloud(CloudPairing::Pair && pair)
  {
    if (!pair.signal) {
      ++unpaired_range_images_;
    }
    emit(point_cloud_, conversions::make_point_cloud(
        pair.range, pair.context.header, true, pair.signal ? &*pair.signal : nullptr));
  }

  template<typename MessageT>
  void emit(Output<MessageT> & output, MessageT && message)
  {
    switch (mode_) {
      case OutputMode::VALIDATE:
        return;
      case OutputMode::BAG:
        writer_->write(message, output.topic, rclcpp::Time(message.header.stamp));
        break;
      case OutputMode::PUBLISH:
        output.publisher->publish(std::make_unique<MessageT>(std::move(message)));
        break;
    }
    ++outputs_[output.topic];
  }

  std::string frame_id_;
  std::string imu_frame_id_;
  bool use_sensor_timestamps_;
  bool intensity_;
  bool vendor_orientation_;
  std::string orientation_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> orientation_tf_;
  std::optional<protocol::ImuOrientation> last_orientation_;
  std::uint64_t missing_orientations_{};
  OutputMode mode_;
  CloudPairing pairing_;
  std::optional<protocol::MessageHeader> last_range_header_;
  std::optional<protocol::MessageHeader> last_signal_header_;
  std::optional<std::int64_t> latest_image_stamp_;
  std::uint64_t unpaired_range_images_{};
  std::map<std::string, std::uint64_t> outputs_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  Output<sensor_msgs::msg::Image> range_image_;
  Output<sensor_msgs::msg::Image> intensity_image_;
  Output<sensor_msgs::msg::Image> shaded_image_;
  Output<sensor_msgs::msg::PointCloud2> point_cloud_;
  Output<sensor_msgs::msg::Imu> imu_;
  Output<sensor_msgs::msg::Imu> orientation_;
  Output<msg::ImageMetadata> image_metadata_;
  Output<msg::RawPacket> raw_packet_;
  std::optional<std::int64_t> raw_stamp_;
  std::uint64_t raw_packet_index_{};
};

// Unknown packets have no interpreted acquisition clock. Anchor their bag
// event times to the first known stamp, then the last known stamp, rather than
// mixing the sonar epoch with this computer's current wall clock.
std::optional<std::int64_t> first_recorded_stamp(std::istream & stream)
{
  std::optional<std::int64_t> stamp;
  const auto origin = stream.tellg();
  while (!stamp) {
    try {
      const auto packet = protocol::read_packet(stream);
      if (!packet) {
        break;
      }
      stamp = sensor_time_nanoseconds(packet->message);
      if (stamp && *stamp < 0) {
        stamp.reset();
      }
    } catch (const protocol::ProtocolError & error) {
      if (!error.framing_intact()) {
        break;
      }
    }
  }
  stream.clear();
  stream.seekg(origin);
  return stamp;
}

[[nodiscard]] int play_recording(const PlaybackOptions & options)
{
  std::ifstream stream(options.file, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("could not open recording: " + options.file);
  }

  auto node = std::make_shared<RecordingPlayer>(options);
  if (options.use_sensor_timestamps && node->mode() != OutputMode::VALIDATE) {
    node->recording_origin(first_recorded_stamp(stream));
  }
  const bool paced = node->mode() == OutputMode::PUBLISH;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  if (paced) {
    interruptible_sleep(options.startup_delay);
  }

  PlaybackClock playback_clock(options.realtime_factor);
  std::uint64_t supported{};
  std::uint64_t unsupported{};
  std::uint64_t damaged{};
  std::uint64_t malformed{};
  std::uint64_t framing_errors{};
  bool reached_eof{false};
  std::map<std::string, std::uint64_t> unsupported_types;
  while (rclcpp::ok()) {
    const auto offset = stream.tellg();
    std::optional<protocol::DecodedPacket> packet;
    std::vector<std::uint8_t> raw_packet;
    try {
      packet = protocol::read_packet(stream, node->wants_raw_packets() ? &raw_packet : nullptr);
    } catch (const protocol::ProtocolError & error) {
      node->process_packet(std::move(raw_packet));
      if (error.framing_intact()) {
        ++damaged;
        RCLCPP_WARN(
          node->get_logger(), "Skipping damaged RIP packet at byte %" PRId64 ": %s",
          static_cast<std::int64_t>(offset), error.what());
        continue;
      }
      ++framing_errors;
      RCLCPP_ERROR(
        node->get_logger(), "Lost RIP framing at byte %" PRId64 ": %s",
        static_cast<std::int64_t>(offset), error.what());
      break;
    }
    if (!packet) {
      reached_eof = true;
      break;
    }

    if (const auto type = unsupported_type(packet->message)) {
      node->process_packet(std::move(raw_packet), sensor_time_nanoseconds(packet->message));
      ++unsupported;
      ++unsupported_types[*type];
      continue;
    }
    try {
      validate_recorded_message(packet->message, options.use_sensor_timestamps);
    } catch (const std::exception & error) {
      node->process_packet(std::move(raw_packet), sensor_time_nanoseconds(packet->message));
      ++malformed;
      RCLCPP_WARN(node->get_logger(), "Skipping malformed decoded message: %s", error.what());
      continue;
    }
    if (paced) {
      try {
        node->wait_until(playback_clock.deadline(
            sensor_time_nanoseconds(packet->message), std::chrono::steady_clock::now()), executor);
      } catch (const std::exception & error) {
        ++malformed;
        RCLCPP_WARN(node->get_logger(), "Skipping malformed waiting cloud: %s", error.what());
      }
    }
    if (!rclcpp::ok()) {
      break;
    }

    node->process_packet(std::move(raw_packet), sensor_time_nanoseconds(packet->message));
    try {
      node->process_message(std::move(packet->message));
    } catch (const std::exception & error) {
      ++malformed;
      RCLCPP_WARN(node->get_logger(), "Skipping malformed decoded message: %s", error.what());
      continue;
    }
    ++supported;
    if (rclcpp::ok()) {
      executor.spin_some();
    }
  }

  try {
    node->flush();
  } catch (const std::exception & error) {
    ++malformed;
    RCLCPP_WARN(node->get_logger(), "Skipping malformed decoded message: %s", error.what());
  }
  // Give reliable DDS writers a brief opportunity to flush their final sample.
  if (paced) {
    interruptible_sleep(0.1);
  }
  if (rclcpp::ok()) {
    executor.spin_some();
  }
  node->finish();
  RCLCPP_INFO(
    node->get_logger(),
    "%s %" PRIu64 " supported packets; unsupported %" PRIu64
    "; damaged %" PRIu64 "; malformed %" PRIu64 "; framing errors %" PRIu64,
    options.validate_only ? "Validated" : "Processed",
    supported, unsupported, damaged, malformed, framing_errors);
  if (node->unpaired_range_images() != 0) {
    RCLCPP_INFO(
      node->get_logger(),
      "%" PRIu64 " range images had no signal-strength image; their points have zero intensity",
      node->unpaired_range_images());
  }
  if (node->missing_orientations() != 0) {
    RCLCPP_WARN(node->get_logger(),
      "%" PRIu64 " range images lack an exact orientation; "
      "omitted from attitude-only cloud preview",
      node->missing_orientations());
  }
  for (const auto & [type, count] : unsupported_types) {
    RCLCPP_INFO(
      node->get_logger(), "Unsupported message type %s: %" PRIu64 " packets (payload not decoded)",
      type.c_str(), count);
  }
  for (const auto & [topic, count] : node->outputs()) {
    RCLCPP_INFO(
      node->get_logger(), "%s %s: %" PRIu64 " ROS messages",
      node->mode() == OutputMode::BAG ? "Wrote" : "Published", topic.c_str(), count);
  }
  if (node->mode() == OutputMode::BAG) {
    RCLCPP_INFO(node->get_logger(), "Bag written to %s", options.output.c_str());
  }
  if (!reached_eof && !framing_errors) {
    RCLCPP_WARN(node->get_logger(), "Stopped before recording EOF; results are incomplete");
  }
  if (framing_errors || (options.validate_only && (damaged || malformed || !reached_eof))) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace sonar3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    const auto options = sonar3d::parse_arguments(arguments);
    if (options.help) {
      sonar3d::print_usage(argv[0]);
      rclcpp::shutdown();
      return EXIT_SUCCESS;
    }
    const auto result = sonar3d::play_recording(options);
    rclcpp::shutdown();
    return result;
  } catch (const std::exception & error) {
    std::cerr << argv[0] << ": " << error.what() << '\n';
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
}
