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
#include "sonar3d/protocol.hpp"

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

[[nodiscard]] std::optional<double> sensor_time_seconds(const protocol::Message & message)
{
  const auto to_seconds = [](const protocol::Timestamp & stamp) -> std::optional<double> {
      if (stamp.seconds == 0 && stamp.nanoseconds == 0) {
        return std::nullopt;
      }
      if (stamp.nanoseconds < 0 || stamp.nanoseconds >= 1'000'000'000) {
        return std::nullopt;
      }
      return static_cast<double>(stamp.seconds) + static_cast<double>(stamp.nanoseconds) / 1.0e9;
    };

  return std::visit(
    [&to_seconds](const auto & value) -> std::optional<double> {
      using MessageType = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<MessageType, protocol::RangeImage>||
      std::is_same_v<MessageType, protocol::BitmapImage>)
      {
        return value.header.timestamp ? to_seconds(*value.header.timestamp) : std::nullopt;
      } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
        return value.timestamps.empty() ? std::nullopt : to_seconds(value.timestamps.front());
      } else {
        return std::nullopt;
      }
    },
    message);
}

void interruptible_sleep(double seconds)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const std::chrono::duration<double> remaining = deadline - std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::min(remaining, std::chrono::duration<double>(0.05)));
  }
}

class RecordingPlayer final : public rclcpp::Node
{
public:
  explicit RecordingPlayer(const PlaybackOptions & options)
  : Node("sonar3d_playback"),
    frame_id_(options.frame_id),
    imu_frame_id_(options.imu_frame_id),
    use_sensor_timestamps_(options.use_sensor_timestamps)
  {
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

  [[nodiscard]] bool publish_message(const protocol::Message & source)
  {
    const builtin_interfaces::msg::Time fallback_stamp = now();
    return std::visit(
      [this, &fallback_stamp](const auto & message) {
        using MessageType = std::decay_t<decltype(message)>;
        if constexpr (std::is_same_v<MessageType, protocol::RangeImage>) {
          const auto header = conversions::make_header(
            message.header,
            frame_id_,
            fallback_stamp,
            use_sensor_timestamps_);
          auto range_image = conversions::make_range_image(message, header);
          auto point_cloud = conversions::make_point_cloud(message, header);
          range_image_publisher_->publish(std::move(range_image));
          point_cloud_publisher_->publish(std::move(point_cloud));
          return true;
        } else if constexpr (std::is_same_v<MessageType, protocol::BitmapImage>) {
          const auto header = conversions::make_header(
            message.header,
            frame_id_,
            fallback_stamp,
            use_sensor_timestamps_);
          auto image = conversions::make_bitmap_image(message, header);
          if (message.type == protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE) {
            intensity_image_publisher_->publish(std::move(image));
            return true;
          }
          if (message.type == protocol::BitmapImageType::SHADED_IMAGE) {
            shaded_image_publisher_->publish(std::move(image));
            return true;
          }
          return false;
        } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
          auto messages = conversions::make_imu_messages(
            message,
            imu_frame_id_,
            fallback_stamp,
            use_sensor_timestamps_);
          for (auto & imu : messages) {
            imu_publisher_->publish(std::move(imu));
          }
          return true;
        } else {
          return false;
        }
      },
      source);
  }

private:
  std::string frame_id_;
  std::string imu_frame_id_;
  bool use_sensor_timestamps_;
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
  interruptible_sleep(options.startup_delay);

  std::optional<double> previous_stamp;
  std::uint64_t published{};
  std::uint64_t skipped{};
  while (rclcpp::ok()) {
    const auto offset = stream.tellg();
    std::optional<protocol::DecodedPacket> packet;
    try {
      packet = protocol::read_packet(stream);
    } catch (const protocol::ProtocolError & error) {
      if (error.framing_intact()) {
        ++skipped;
        RCLCPP_WARN(
          node->get_logger(), "Skipping damaged RIP packet at byte %" PRId64 ": %s",
          static_cast<std::int64_t>(offset), error.what());
        continue;
      }
      throw std::runtime_error(
        "lost RIP framing at byte " + std::to_string(static_cast<std::int64_t>(offset)) + ": " +
            error.what());
    }
    if (!packet) {
      break;
    }

    const auto stamp = sensor_time_seconds(packet->message);
    if (stamp && previous_stamp) {
      const auto delay = (*stamp - *previous_stamp) / options.realtime_factor;
      if (delay > 0.0) {
        interruptible_sleep(delay);
      }
    }

    try {
      if (!node->publish_message(packet->message)) {
        ++skipped;
        continue;
      }
    } catch (const std::exception & error) {
      ++skipped;
      RCLCPP_WARN(node->get_logger(), "Skipping malformed decoded message: %s", error.what());
      continue;
    }
    if (stamp) {
      previous_stamp = stamp;
    }
    ++published;
    rclcpp::spin_some(node);
  }

  // Give reliable DDS writers a brief opportunity to flush their final sample.
  interruptible_sleep(0.1);
  rclcpp::spin_some(node);
  RCLCPP_INFO(
    node->get_logger(),
    "Published %" PRIu64 " Sonar 3D-15 packets; skipped %" PRIu64,
    published,
    skipped);
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
