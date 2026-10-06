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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/playback_clock.hpp"
#include "sonar3d/protocol.hpp"
#include "sonar3d/publisher_demand.hpp"

namespace sonar3d
{
namespace
{

struct PlaybackOptions
{
  std::string file;
  std::string frame_id{"sonar3d_link"};
  std::string imu_frame_id{"sonar3d_imu_link"};
  double realtime_factor{1.0};
  double startup_delay{0.5};
  bool use_sensor_timestamps{true};
  bool validate_only{false};
  bool help{false};
};

void print_usage(const char * executable)
{
  std::cout
    << "Usage: " << executable << " --file RECORDING [options]\n"
    << "\n"
    << "Publish a mixed RIP1/RIP2 .sonar recording using the live driver's topics.\n"
    << "\n"
    << "Options:\n"
    << "  --file PATH                 Recording to play (required)\n"
    << "  --realtime-factor FACTOR   Playback speed multiplier (default: 1.0)\n"
    << "  --frame-id FRAME           Range/image/cloud frame (default: sonar3d_link)\n"
    << "  --imu-frame-id FRAME       IMU frame (default: sonar3d_imu_link)\n"
    << "  --startup-delay SECONDS    DDS discovery delay (default: 0.5)\n"
    << "  --receive-time             Stamp products from the replay ROS clock\n"
    << "  --validate-only            Check all supported conversions without publishing or pacing\n"
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
    } else if (argument == "--validate-only") {
      options.validate_only = true;
    } else if (argument == "--file" || argument.rfind("--file=", 0) == 0) {
      options.file = option_value(arguments, index, "--file");
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
      std::is_same_v<MessageType, protocol::BitmapImage>)
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

class RecordingPlayer final : public rclcpp::Node
{
public:
  explicit RecordingPlayer(const PlaybackOptions & options)
  : Node("sonar3d_playback"),
    frame_id_(options.frame_id),
    imu_frame_id_(options.imu_frame_id),
    use_sensor_timestamps_(options.use_sensor_timestamps),
    validate_only_(options.validate_only)
  {
    if (validate_only_) {
      return;
    }
    auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(100));
    qos.reliable();
    range_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sonar_range_image", qos);
    intensity_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sonar_intensity_image",
          qos);
    shaded_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sonar_shaded_image", qos);
    point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("sonar_point_cloud",
          qos);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("sonar_imu", qos);
  }

  void process_message(const protocol::Message & source)
  {
    const builtin_interfaces::msg::Time fallback_stamp = now();
    std::visit(
      [this, &fallback_stamp](const auto & message) {
        using MessageType = std::decay_t<decltype(message)>;
        if constexpr (std::is_same_v<MessageType, protocol::RangeImage>) {
          const auto header = conversions::make_header(
            message.header,
            frame_id_,
            fallback_stamp,
            use_sensor_timestamps_);
          if (validate_only_ || has_subscribers(range_image_publisher_)) {
            auto image = conversions::make_range_image(message, header);
            if (!validate_only_) {
              range_image_publisher_->publish(
                std::make_unique<sensor_msgs::msg::Image>(std::move(image)));
              ++publications_[range_image_publisher_->get_topic_name()];
            }
          }
          if (validate_only_ || has_subscribers(point_cloud_publisher_)) {
            auto cloud = conversions::make_point_cloud(message, header);
            if (!validate_only_) {
              point_cloud_publisher_->publish(
                std::make_unique<sensor_msgs::msg::PointCloud2>(std::move(cloud)));
              ++publications_[point_cloud_publisher_->get_topic_name()];
            }
          }
        } else if constexpr (std::is_same_v<MessageType, protocol::BitmapImage>) {
          rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
          if (message.type == protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE) {
            publisher = intensity_image_publisher_;
          } else if (message.type == protocol::BitmapImageType::SHADED_IMAGE) {
            publisher = shaded_image_publisher_;
          } else {
            return;
          }
          if (validate_only_ || has_subscribers(publisher)) {
            const auto header = conversions::make_header(
              message.header, frame_id_, fallback_stamp, use_sensor_timestamps_);
            auto image = conversions::make_bitmap_image(message, header);
            if (!validate_only_) {
              publisher->publish(std::make_unique<sensor_msgs::msg::Image>(std::move(image)));
              ++publications_[publisher->get_topic_name()];
            }
          }
        } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
          if (validate_only_ || has_subscribers(imu_publisher_)) {
            auto messages = conversions::make_imu_messages(
              message, imu_frame_id_, fallback_stamp, use_sensor_timestamps_);
            if (!validate_only_) {
              for (auto & imu : messages) {
                imu_publisher_->publish(std::make_unique<sensor_msgs::msg::Imu>(std::move(imu)));
                ++publications_[imu_publisher_->get_topic_name()];
              }
            }
          }
        }
      },
      source);
  }

  const std::map<std::string, std::uint64_t> & publications() const {return publications_;}

private:
  std::string frame_id_;
  std::string imu_frame_id_;
  bool use_sensor_timestamps_;
  bool validate_only_;
  std::map<std::string, std::uint64_t> publications_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr range_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr intensity_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr shaded_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

[[nodiscard]] int play_recording(const PlaybackOptions & options)
{
  std::ifstream stream(options.file, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("could not open recording: " + options.file);
  }

  auto node = std::make_shared<RecordingPlayer>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  if (!options.validate_only) {
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
    try {
      packet = protocol::read_packet(stream);
    } catch (const protocol::ProtocolError & error) {
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
      ++unsupported;
      ++unsupported_types[*type];
      continue;
    }
    if (!options.validate_only) {
      interruptible_sleep_until(playback_clock.deadline(
          sensor_time_nanoseconds(packet->message), std::chrono::steady_clock::now()));
    }
    if (!rclcpp::ok()) {
      break;
    }

    try {
      node->process_message(packet->message);
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

  // Give reliable DDS writers a brief opportunity to flush their final sample.
  if (!options.validate_only) {
    interruptible_sleep(0.1);
  }
  if (rclcpp::ok()) {
    executor.spin_some();
  }
  RCLCPP_INFO(
    node->get_logger(),
    "%s %" PRIu64 " supported packets; unsupported %" PRIu64
    "; damaged %" PRIu64 "; malformed %" PRIu64 "; framing errors %" PRIu64,
    options.validate_only ? "Validated" : "Processed",
    supported, unsupported, damaged, malformed, framing_errors);
  for (const auto & [type, count] : unsupported_types) {
    RCLCPP_INFO(
      node->get_logger(), "Unsupported message type %s: %" PRIu64 " packets (payload not decoded)",
      type.c_str(), count);
  }
  for (const auto & [topic, count] : node->publications()) {
    RCLCPP_INFO(node->get_logger(), "Published %s: %" PRIu64 " ROS messages", topic.c_str(), count);
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
