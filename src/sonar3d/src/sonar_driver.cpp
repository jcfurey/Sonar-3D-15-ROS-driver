// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "sonar3d/sonar_driver.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "sonar3d/conversions.hpp"
#include "sonar3d/http_client.hpp"
#include "sonar3d/publisher_demand.hpp"

namespace sonar3d
{
namespace
{

[[nodiscard]] std::int64_t steady_time_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

[[nodiscard]] bool is_ipv4_address(const std::string & value)
{
  in_addr address{};
  return inet_pton(AF_INET, value.c_str(), &address) == 1;
}

[[nodiscard]] std::string join(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  return stream.str();
}

void add_diagnostic_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  status.values.push_back(std::move(item));
}

template<typename Value>
void add_diagnostic_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  Value value)
{
  add_diagnostic_value(status, key, std::to_string(value));
}

}  // namespace

SonarDriver::SonarDriver(const rclcpp::NodeOptions & options)
: Node("sonar3d_driver", options)
{
  parameter_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  const auto parameters = parameter_listener_->get_params();

  validate_configuration(parameters.speed_of_sound, parameters.http_timeout);
  if (parameters.configure_sonar && parameters.sonar_ip.empty()) {
    throw std::invalid_argument("sonar_ip must not be empty when configure_sonar is true");
  }
  if (!parameters.sonar_ip.empty() && !is_ipv4_address(parameters.sonar_ip)) {
    throw std::invalid_argument(
            "sonar_ip must be a literal IPv4 address so UDP source filtering is unambiguous");
  }
  if (parameters.frame_id.empty() || parameters.imu_frame_id.empty()) {
    throw std::invalid_argument("frame_id and imu_frame_id must not be empty");
  }

  sonar_ip_ = parameters.sonar_ip;
  frame_id_ = parameters.frame_id;
  imu_frame_id_ = parameters.imu_frame_id;
  use_sensor_timestamps_ = parameters.use_sensor_timestamps;
  publish_point_cloud_ = parameters.publish_point_cloud;
  publish_range_image_ = parameters.publish_range_image;
  publish_bitmap_images_ = parameters.publish_bitmap_images;
  publish_imu_ = parameters.publish_imu;
  max_packets_per_spin_ = static_cast<std::size_t>(parameters.max_packets_per_spin);
  packet_stale_timeout_ = parameters.packet_stale_timeout;

  receiver_ = std::make_unique<UdpReceiver>(
    parameters.multicast_group,
    static_cast<std::uint16_t>(parameters.multicast_port),
    parameters.multicast_interface,
    static_cast<int>(parameters.udp_receive_buffer_size));

  const auto sensor_qos = rclcpp::SensorDataQoS();
  if (publish_range_image_) {
    range_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sonar_range_image",
        sensor_qos);
  }
  if (publish_point_cloud_) {
    point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("sonar_point_cloud", sensor_qos);
  }
  if (publish_bitmap_images_) {
    intensity_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("sonar_intensity_image", sensor_qos);
    shaded_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sonar_shaded_image",
        sensor_qos);
  }
  if (publish_imu_) {
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("sonar_imu", sensor_qos);
  }
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));

  poll_timer_ = create_wall_timer(
    std::chrono::duration<double>(parameters.poll_period),
    std::bind(&SonarDriver::poll_socket, this));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(parameters.diagnostics_period),
    std::bind(&SonarDriver::publish_diagnostics, this));

  RCLCPP_INFO(
    get_logger(),
    "Listening for Sonar 3D-15 RIP1/RIP2 packets on %s:%ld via interface %s",
    parameters.multicast_group.c_str(),
    parameters.multicast_port,
    parameters.multicast_interface.c_str());
  if (!sonar_ip_.empty()) {
    RCLCPP_INFO(get_logger(), "Accepting packets only from %s", sonar_ip_.c_str());
  }

  if (parameters.configure_sonar) {
    start_configuration(parameters.speed_of_sound, parameters.http_timeout);
  }
}

SonarDriver::~SonarDriver()
{
  if (poll_timer_) {
    poll_timer_->cancel();
  }
  if (diagnostics_timer_) {
    diagnostics_timer_->cancel();
  }
  if (configuration_thread_.joinable()) {
    configuration_thread_.request_stop();
    configuration_thread_.join();
  }
  receiver_.reset();
}

void SonarDriver::start_configuration(double speed_of_sound, double timeout_seconds)
{
  configuration_state_.store(1);
  const auto timeout = std::chrono::milliseconds(
    static_cast<std::int64_t>(std::llround(timeout_seconds * 1000.0)));
  configuration_thread_ = std::jthread(
    [this, speed_of_sound, timeout](std::stop_token stop_token) {
      try {
        HttpSonarApi api(sonar_ip_, timeout);
        const auto applied = configure_sonar(
          api,
          speed_of_sound,
          [&stop_token]() {return stop_token.stop_requested();});
        if (stop_token.stop_requested()) {
          configuration_state_.store(0);
          return;
        }
        configuration_state_.store(2);
        RCLCPP_INFO(get_logger(), "Configured sonar settings: %s", join(applied).c_str());
      } catch (const std::exception & error) {
        {
          std::lock_guard<std::mutex> lock(configuration_error_mutex_);
          configuration_error_ = error.what();
        }
        configuration_state_.store(3);
        RCLCPP_WARN(
          get_logger(),
          "Could not configure Sonar 3D-15 at %s: %s",
          sonar_ip_.c_str(),
          error.what());
      }
    });
}

void SonarDriver::poll_socket()
{
  for (std::size_t packet_index = 0; packet_index < max_packets_per_spin_; ++packet_index) {
    try {
      const auto datagram = receiver_->receive();
      if (!datagram) {
        return;
      }
      metrics_.datagrams.fetch_add(1, std::memory_order_relaxed);
      handle_datagram(*datagram);
    } catch (const std::system_error & error) {
      metrics_.socket_errors.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Sonar multicast receive failed: %s",
        error.what());
      return;
    }
  }
}

void SonarDriver::handle_datagram(const Datagram & datagram)
{
  if (!sonar_ip_.empty() && datagram.source_address != sonar_ip_) {
    metrics_.rejected_sources.fetch_add(1, std::memory_order_relaxed);
    if (warned_sources_.insert(datagram.source_address).second) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring Sonar 3D-15 packets from %s; expected %s",
        datagram.source_address.c_str(),
        sonar_ip_.c_str());
    }
    return;
  }

  protocol::DecodedPacket packet{protocol::ProtocolVersion::RIP1, protocol::UnknownMessage{}};
  try {
    packet = protocol::decode_packet(datagram.bytes);
  } catch (const protocol::ProtocolError & error) {
    metrics_.malformed_packets.fetch_add(1, std::memory_order_relaxed);
    if (error.code() == protocol::ErrorCode::CRC_MISMATCH) {
      metrics_.crc_failures.fetch_add(1, std::memory_order_relaxed);
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Rejected malformed RIP packet: %s",
      error.what());
    return;
  }

  metrics_.valid_packets.fetch_add(1, std::memory_order_relaxed);
  last_valid_packet_steady_ns_.store(steady_time_nanoseconds(), std::memory_order_relaxed);
  const builtin_interfaces::msg::Time fallback_stamp = now();

  try {
    std::visit(
      [this, &fallback_stamp](const auto & message) {
        using MessageType = std::decay_t<decltype(message)>;
        if constexpr (std::is_same_v<MessageType, protocol::RangeImage>) {
          handle_range_image(message, fallback_stamp);
        } else if constexpr (std::is_same_v<MessageType, protocol::BitmapImage>) {
          handle_bitmap_image(message, fallback_stamp);
        } else if constexpr (std::is_same_v<MessageType, protocol::ImuBatch>) {
          handle_imu_batch(message, fallback_stamp);
        } else {
          metrics_.unknown_messages.fetch_add(1, std::memory_order_relaxed);
          RCLCPP_DEBUG(get_logger(), "Ignoring unsupported RIP message %s",
            message.type_url.c_str());
        }
      },
      packet.message);
  } catch (const std::exception & error) {
    metrics_.malformed_packets.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Rejected malformed Sonar 3D-15 message: %s",
      error.what());
  }
}

void SonarDriver::handle_range_image(
  const protocol::RangeImage & image,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  observe_sequence(range_sequence_, image.header.sequence_id);
  const auto header = conversions::make_header(
    image.header,
    frame_id_,
    fallback_stamp,
    use_sensor_timestamps_);

  std::optional<sensor_msgs::msg::Image> range_message;
  std::optional<sensor_msgs::msg::PointCloud2> cloud_message;
  if (has_subscribers(range_image_publisher_)) {
    range_message = conversions::make_range_image(image, header);
  }
  if (has_subscribers(point_cloud_publisher_)) {
    cloud_message = conversions::make_point_cloud(image, header);
  }
  if (range_message) {
    range_image_publisher_->publish(
      std::make_unique<sensor_msgs::msg::Image>(std::move(*range_message)));
  }
  if (cloud_message) {
    point_cloud_publisher_->publish(
      std::make_unique<sensor_msgs::msg::PointCloud2>(std::move(*cloud_message)));
  }
  metrics_.range_images.fetch_add(1, std::memory_order_relaxed);
}

void SonarDriver::handle_bitmap_image(
  const protocol::BitmapImage & image,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  if (!publish_bitmap_images_) {
    metrics_.bitmap_images.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  switch (image.type) {
    case protocol::BitmapImageType::SIGNAL_STRENGTH_IMAGE:
      observe_sequence(intensity_sequence_, image.header.sequence_id);
      publisher = intensity_image_publisher_;
      break;
    case protocol::BitmapImageType::SHADED_IMAGE:
      observe_sequence(shaded_sequence_, image.header.sequence_id);
      publisher = shaded_image_publisher_;
      break;
    default:
      metrics_.unknown_messages.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Ignoring bitmap with unknown image type %d",
        static_cast<int>(image.type));
      return;
  }
  if (has_subscribers(publisher)) {
    const auto header = conversions::make_header(
      image.header, frame_id_, fallback_stamp, use_sensor_timestamps_);
    publisher->publish(std::make_unique<sensor_msgs::msg::Image>(
        conversions::make_bitmap_image(image, header)));
  }
  metrics_.bitmap_images.fetch_add(1, std::memory_order_relaxed);
}

void SonarDriver::handle_imu_batch(
  const protocol::ImuBatch & batch,
  const builtin_interfaces::msg::Time & fallback_stamp)
{
  observe_sequence(imu_sequence_, batch.sequence_id);
  if (has_subscribers(imu_publisher_)) {
    auto messages = conversions::make_imu_messages(
      batch,
      imu_frame_id_,
      fallback_stamp,
      use_sensor_timestamps_);
    for (auto & message : messages) {
      imu_publisher_->publish(std::make_unique<sensor_msgs::msg::Imu>(std::move(message)));
    }
  }
  metrics_.imu_batches.fetch_add(1, std::memory_order_relaxed);
}

void SonarDriver::observe_sequence(SequenceState & state, std::uint32_t sequence_id)
{
  if (!state.last) {
    state.last = sequence_id;
    return;
  }

  const auto previous = *state.last;
  const auto forward_delta = sequence_id - previous;
  if (forward_delta == 0) {
    metrics_.duplicate_messages.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (forward_delta < (std::uint32_t{1} << 31U)) {
    metrics_.sequence_gaps.fetch_add(forward_delta - 1U, std::memory_order_relaxed);
    state.last = sequence_id;
    return;
  }

  const auto backward_delta = previous - sequence_id;
  constexpr std::uint32_t kMaximumReorderingDistance = 64;
  if (backward_delta > kMaximumReorderingDistance) {
    metrics_.sequence_resets.fetch_add(1, std::memory_order_relaxed);
    state.last = sequence_id;
  } else {
    metrics_.out_of_order_messages.fetch_add(1, std::memory_order_relaxed);
  }
}

void SonarDriver::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(get_fully_qualified_name()) + ": RIP stream";
  status.hardware_id = sonar_ip_.empty() ? "any Sonar 3D-15" : sonar_ip_;

  const auto last_packet = last_valid_packet_steady_ns_.load(std::memory_order_relaxed);
  double packet_age = -1.0;
  if (last_packet < 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "waiting for valid RIP packets";
  } else {
    packet_age = static_cast<double>(steady_time_nanoseconds() - last_packet) / 1.0e9;
    if (packet_age > packet_stale_timeout_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "RIP packet stream is stale";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "receiving RIP packets";
    }
  }

  const auto configuration_state = configuration_state_.load(std::memory_order_relaxed);
  std::string configuration_text;
  switch (configuration_state) {
    case 0:
      configuration_text = "disabled";
      break;
    case 1:
      configuration_text = "pending";
      break;
    case 2:
      configuration_text = "successful";
      break;
    case 3:
      configuration_text = "failed";
      status.level = std::max(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
      break;
    default:
      configuration_text = "unknown";
      break;
  }

  add_diagnostic_value(status, "configuration", configuration_text);
  if (configuration_state == 3) {
    std::lock_guard<std::mutex> lock(configuration_error_mutex_);
    add_diagnostic_value(status, "configuration_error", configuration_error_);
  }
  add_diagnostic_value(status, "last_packet_age_seconds", packet_age);
  add_diagnostic_value(status, "udp_receive_buffer_bytes", receiver_->receive_buffer_size());
  add_diagnostic_value(status, "datagrams", metrics_.datagrams.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "rejected_sources",
      metrics_.rejected_sources.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "valid_packets",
      metrics_.valid_packets.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "malformed_packets",
      metrics_.malformed_packets.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "crc_failures",
      metrics_.crc_failures.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "unknown_messages",
      metrics_.unknown_messages.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "range_images",
      metrics_.range_images.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "bitmap_images",
      metrics_.bitmap_images.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "imu_batches", metrics_.imu_batches.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "sequence_gaps",
      metrics_.sequence_gaps.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "duplicates",
      metrics_.duplicate_messages.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "out_of_order",
      metrics_.out_of_order_messages.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "sequence_resets",
      metrics_.sequence_resets.load(std::memory_order_relaxed));
  add_diagnostic_value(status, "socket_errors",
      metrics_.socket_errors.load(std::memory_order_relaxed));

  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
}

}  // namespace sonar3d

RCLCPP_COMPONENTS_REGISTER_NODE(sonar3d::SonarDriver)
