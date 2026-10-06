// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "sonar3d/sonar_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "sonar3d/http_client.hpp"
#include "sonar3d/publisher_demand.hpp"

namespace sonar3d
{
namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

[[nodiscard]] std::int64_t steady_time_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
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

[[nodiscard]] std::uint64_t value(const std::atomic<std::uint64_t> & counter)
{
  return counter.load(std::memory_order_relaxed);
}

}  // namespace

SonarDriver::SonarDriver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sonar3d_driver", options),
  range_rate_(
    diagnostic_updater::FrequencyStatusParam(&minimum_range_rate_, &maximum_range_rate_),
    "Range image rate"),
  diagnostics_(this)
{
  // Parameters are declared here so they can be set before configure, as the
  // lifecycle pattern expects; they are read and validated in on_configure.
  const auto inherited = list_parameters({}, 0).names;
  parameter_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  for (const auto & name : list_parameters({}, 0).names) {
    if (std::find(inherited.begin(), inherited.end(), name) == inherited.end()) {
      driver_parameters_.insert(name);
    }
  }
  parameter_guard_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      const auto state = get_current_state().id();
      if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        return result;
      }
      for (const auto & parameter : parameters) {
        if (driver_parameters_.contains(parameter.get_name())) {
          result.successful = false;
          result.reason = parameter.get_name() + " can only change while the driver is "
          "unconfigured; deactivate and clean up the driver, set it, then configure";
          break;
        }
      }
      return result;
    });
  diagnostics_.setHardwareID("Sonar 3D-15");
  diagnostics_.add("RIP stream", this, &SonarDriver::diagnose_stream);

  if (parameter_listener_->get_params().autostart) {
    // Transition once the executor spins the node, so a component container
    // has finished loading it first.
    autostart_timer_ = create_wall_timer(std::chrono::nanoseconds(0), [this]() {
          autostart_timer_->cancel();
          if (configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            activate();
          }
        });
  }
}

SonarDriver::~SonarDriver()
{
  stop_streaming();
  stop_configuration();
}

SonarDriver::CallbackReturn SonarDriver::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    configure_driver();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Could not configure the Sonar 3D-15 driver: %s", error.what());
    release();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

void SonarDriver::configure_driver()
{
  const auto parameters = parameter_listener_->get_params();
  validate_configuration(parameters.speed_of_sound, parameters.http_timeout);
  if (parameters.configure_sonar && parameters.sonar_ip.empty()) {
    throw std::invalid_argument("sonar_ip must not be empty when configure_sonar is true");
  }

  Settings settings;
  settings.sonar_ip = parameters.sonar_ip;
  settings.fallback_ip = parameters.fallback_ip;
  if (!settings.sonar_ip.empty()) {
    settings.sonar_address = parse_ipv4(settings.sonar_ip);
    if (!settings.sonar_address) {
      throw std::invalid_argument(
              "sonar_ip must be a literal IPv4 address so UDP source filtering is unambiguous");
    }
  }
  if (!settings.fallback_ip.empty()) {
    settings.fallback_address = parse_ipv4(settings.fallback_ip);
    if (!settings.fallback_address) {
      throw std::invalid_argument("fallback_ip must be empty or a literal IPv4 address");
    }
  }
  if (parameters.frame_id.empty() || parameters.imu_frame_id.empty()) {
    throw std::invalid_argument("frame_id and imu_frame_id must not be empty");
  }
  settings.frame_id = parameters.frame_id;
  settings.imu_frame_id = parameters.imu_frame_id;
  settings.use_sensor_timestamps = parameters.use_sensor_timestamps;
  settings.max_sensor_clock_offset_ns = std::llround(parameters.max_sensor_clock_offset * 1.0e9);
  settings.imu_noise = {parameters.linear_acceleration_stddev, parameters.angular_velocity_stddev};
  settings.packet_stale_timeout = parameters.packet_stale_timeout;
  settings_ = std::move(settings);

  receiver_ = std::make_unique<UdpReceiver>(
    parameters.multicast_group,
    static_cast<std::uint16_t>(parameters.multicast_port),
    parameters.multicast_interface,
    static_cast<int>(parameters.udp_receive_buffer_size));

  // REP-2003: drivers publish with SystemDefaultsQoS (reliable) so that both
  // reliable subscribers and SensorDataQoS (best-effort) subscribers connect.
  // IMU samples arrive in batches of up to 20, so keep a second of history.
  const auto product_qos = rclcpp::SystemDefaultsQoS().keep_last(5);
  if (parameters.publish_range_image) {
    range_image_publisher_ = create_publisher<sensor_msgs::msg::Image>("range_image", product_qos);
  }
  if (parameters.publish_point_cloud) {
    point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", product_qos);
  }
  if (parameters.publish_bitmap_images) {
    intensity_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("intensity_image", product_qos);
    shaded_image_publisher_ =
      create_publisher<sensor_msgs::msg::Image>("shaded_image", product_qos);
  }
  if (parameters.publish_imu) {
    // REP-145: accelerometer and gyroscope samples without orientation.
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu/data_raw", rclcpp::SystemDefaultsQoS().keep_last(100));
  }
  if (parameters.publish_tf) {
    static_transform_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publish_imu_transform();
  }
  diagnostics_.setHardwareID(settings_.sonar_ip.empty() ? "any Sonar 3D-15" : settings_.sonar_ip);

  RCLCPP_INFO(
    get_logger(),
    "Listening for Sonar 3D-15 RIP1/RIP2 packets on %s:%ld via interface %s",
    parameters.multicast_group.c_str(),
    parameters.multicast_port,
    parameters.multicast_interface.c_str());
  if (settings_.sonar_address && settings_.fallback_address &&
    *settings_.sonar_address != *settings_.fallback_address)
  {
    RCLCPP_INFO(
      get_logger(), "Accepting packets only from %s and the fallback address %s",
      settings_.sonar_ip.c_str(), settings_.fallback_ip.c_str());
  } else if (settings_.sonar_address) {
    RCLCPP_INFO(get_logger(), "Accepting packets only from %s", settings_.sonar_ip.c_str());
  }

  if (parameters.configure_sonar) {
    start_configuration(parameters.speed_of_sound, parameters.publish_imu,
      parameters.http_timeout);
  }
}

SonarDriver::CallbackReturn SonarDriver::on_activate(const rclcpp_lifecycle::State & state)
{
  // Activate the publishers before the receive thread can use them.
  rclcpp_lifecycle::LifecycleNode::on_activate(state);
  range_sequence_ = {};
  intensity_sequence_ = {};
  shaded_sequence_ = {};
  imu_sequence_ = {};
  clock_offset_.reset();
  timestamp_source_.store(TimestampSource::NONE);
  last_valid_packet_steady_ns_.store(-1);
  // Datagrams queued while inactive are stale; start from the live stream.
  receiver_->clear_interrupt();
  const auto discarded = receiver_->discard_pending();
  if (discarded != 0) {
    RCLCPP_DEBUG(get_logger(), "Discarded %zu datagrams queued while inactive", discarded);
  }
  range_rate_.clear();
  diagnostics_.add(range_rate_);
  streaming_.store(true);
  receive_thread_ = std::jthread([this](std::stop_token stop) {receive_loop(stop);});
  return CallbackReturn::SUCCESS;
}

SonarDriver::CallbackReturn SonarDriver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Stop publishing before the publishers are deactivated.
  stop_streaming();
  diagnostics_.removeByName(range_rate_.getName());
  rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

SonarDriver::CallbackReturn SonarDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  release();
  return CallbackReturn::SUCCESS;
}

SonarDriver::CallbackReturn SonarDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_streaming();
  diagnostics_.removeByName(range_rate_.getName());
  release();
  return CallbackReturn::SUCCESS;
}

SonarDriver::CallbackReturn SonarDriver::on_error(const rclcpp_lifecycle::State &)
{
  stop_streaming();
  diagnostics_.removeByName(range_rate_.getName());
  release();
  // Recovered: the node returns to unconfigured.
  return CallbackReturn::SUCCESS;
}

void SonarDriver::stop_streaming()
{
  if (receive_thread_.joinable()) {
    receive_thread_.request_stop();
    receiver_->interrupt();
    receive_thread_.join();
  }
  streaming_.store(false);
}

void SonarDriver::stop_configuration()
{
  if (configuration_thread_.joinable()) {
    configuration_thread_.request_stop();
    configuration_thread_.join();
  }
}

void SonarDriver::release()
{
  stop_configuration();
  configuration_state_.store(ConfigurationState::DISABLED);
  static_transform_broadcaster_.reset();
  range_image_publisher_.reset();
  intensity_image_publisher_.reset();
  shaded_image_publisher_.reset();
  point_cloud_publisher_.reset();
  imu_publisher_.reset();
  receiver_.reset();
}

void SonarDriver::publish_imu_transform()
{
  // Water Linked documents the IMU at (-22, -46, -3) mm from the point-cloud
  // origin in its x-forward, y-right, z-down frame. Both frames use REP-103
  // axes here, so y and z change sign and no rotation is needed.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = settings_.frame_id;
  transform.child_frame_id = settings_.imu_frame_id;
  transform.transform.translation.x = -0.022;
  transform.transform.translation.y = 0.046;
  transform.transform.translation.z = 0.003;
  transform.transform.rotation.w = 1.0;
  static_transform_broadcaster_->sendTransform(transform);
}

void SonarDriver::start_configuration(
  double speed_of_sound, bool imu_output, double timeout_seconds)
{
  {
    std::lock_guard<std::mutex> lock(configuration_mutex_);
    configuration_error_.clear();
    configuration_unsupported_.clear();
  }
  configuration_state_.store(ConfigurationState::PENDING);
  const auto timeout = std::chrono::milliseconds(
    static_cast<std::int64_t>(std::llround(timeout_seconds * 1000.0)));
  configuration_thread_ = std::jthread(
    [this, speed_of_sound, imu_output, timeout, sonar_ip = settings_.sonar_ip](
      std::stop_token stop_token) {
      try {
        HttpSonarApi api(sonar_ip, timeout);
        const auto result = configure_sonar(
          api,
          {speed_of_sound, imu_output},
          [&stop_token]() {return stop_token.stop_requested();});
        if (stop_token.stop_requested()) {
          configuration_state_.store(ConfigurationState::DISABLED);
          return;
        }
        {
          std::lock_guard<std::mutex> lock(configuration_mutex_);
          configuration_unsupported_ = result.unsupported;
        }
        configuration_state_.store(ConfigurationState::SUCCESSFUL);
        RCLCPP_INFO(get_logger(), "Configured sonar settings: %s", join(result.applied).c_str());
        if (!result.unsupported.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "The sonar firmware does not provide public ImuBatch output (sonar release 1.8.0 or "
            "newer is required), so imu/data_raw stays silent. Set publish_imu:=false to "
            "silence this warning.");
        }
      } catch (const std::exception & error) {
        {
          std::lock_guard<std::mutex> lock(configuration_mutex_);
          configuration_error_ = error.what();
        }
        configuration_state_.store(ConfigurationState::FAILED);
        RCLCPP_WARN(
          get_logger(), "Could not configure Sonar 3D-15 at %s: %s", sonar_ip.c_str(),
          error.what());
      }
    });
}

void SonarDriver::receive_loop(const std::stop_token & stop)
{
  while (!stop.stop_requested()) {
    try {
      if (!receiver_->wait(std::chrono::milliseconds(-1))) {
        continue;
      }
      while (!stop.stop_requested()) {
        const auto datagram = receiver_->receive();
        if (!datagram) {
          break;
        }
        metrics_.datagrams.fetch_add(1, std::memory_order_relaxed);
        handle_datagram(*datagram);
      }
    } catch (const std::exception & error) {
      metrics_.socket_errors.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Sonar multicast receive failed: %s",
        error.what());
      // Avoid spinning on a persistent socket error.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

bool SonarDriver::accepts_source(std::uint32_t source_address) const
{
  return !settings_.sonar_address || source_address == *settings_.sonar_address ||
         source_address == settings_.fallback_address;
}

void SonarDriver::handle_datagram(const Datagram & datagram)
{
  if (!accepts_source(datagram.source_address)) {
    metrics_.rejected_sources.fetch_add(1, std::memory_order_relaxed);
    if (warned_sources_.insert(datagram.source_address).second) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring Sonar 3D-15 packets from %s; expected %s",
        format_ipv4(datagram.source_address).c_str(),
        settings_.sonar_ip.c_str());
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
  range_rate_.tick();
  const auto header = conversions::make_header(
    image.header,
    settings_.frame_id,
    fallback_stamp,
    settings_.use_sensor_timestamps,
    sensor_clock_offset(image.header.timestamp, fallback_stamp));

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
  Publisher<sensor_msgs::msg::Image> publisher;
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
  const auto offset = sensor_clock_offset(image.header.timestamp, fallback_stamp);
  if (has_subscribers(publisher)) {
    const auto header = conversions::make_header(
      image.header, settings_.frame_id, fallback_stamp, settings_.use_sensor_timestamps, offset);
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
  // The newest sample has the least transport delay, so it best constrains the
  // clock offset; every sample shares that offset and keeps its spacing.
  const auto offset = sensor_clock_offset(
    batch.timestamps.empty() ? std::nullopt : std::optional{batch.timestamps.back()},
    fallback_stamp);
  if (has_subscribers(imu_publisher_)) {
    auto messages = conversions::make_imu_messages(
      batch,
      settings_.imu_frame_id,
      fallback_stamp,
      settings_.use_sensor_timestamps,
      offset,
      settings_.imu_noise);
    for (auto & message : messages) {
      imu_publisher_->publish(std::make_unique<sensor_msgs::msg::Imu>(std::move(message)));
    }
  }
  metrics_.imu_batches.fetch_add(1, std::memory_order_relaxed);
}

std::int64_t SonarDriver::sensor_clock_offset(
  const std::optional<protocol::Timestamp> & reference,
  const builtin_interfaces::msg::Time & receive_stamp)
{
  const auto sensor = conversions::sensor_nanoseconds(reference);
  if (!settings_.use_sensor_timestamps || !sensor) {
    return 0;
  }

  // Both operands are within the int32-second ROS range, so this cannot overflow.
  const auto receive = rclcpp::Time(receive_stamp).nanoseconds();
  const auto offset = receive - *sensor;
  last_sensor_clock_offset_ns_.store(offset, std::memory_order_relaxed);
  const bool synchronized = settings_.max_sensor_clock_offset_ns == 0 ||
    std::llabs(offset) <= settings_.max_sensor_clock_offset_ns;
  const auto source = synchronized ? TimestampSource::SENSOR : TimestampSource::RECEIVE;
  if (timestamp_source_.exchange(source) != source) {
    if (synchronized) {
      RCLCPP_INFO(
        get_logger(),
        "Sonar 3D-15 clock agrees with ROS time (receive offset %.3f s); using sensor timestamps",
        static_cast<double>(offset) / 1.0e9);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Sonar 3D-15 clock differs from ROS time by %.3f s; shifting sensor timestamps onto ROS "
        "time until it synchronizes. The sonar keeps an arbitrary clock until NTP succeeds, so "
        "give it a reachable NTP server.",
        static_cast<double>(offset) / 1.0e9);
    }
  }
  if (synchronized) {
    clock_offset_.reset();
    return 0;
  }
  metrics_.sensor_clock_fallbacks.fetch_add(1, std::memory_order_relaxed);
  return clock_offset_.update(offset, receive);
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

void SonarDriver::diagnose_stream(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto & lifecycle_state = get_current_state();
  status.add("lifecycle_state", lifecycle_state.label());
  if (!streaming_.load()) {
    status.summary(DiagnosticStatus::OK, "not streaming (lifecycle state " +
      lifecycle_state.label() + ")");
  } else {
    const auto last_packet = last_valid_packet_steady_ns_.load(std::memory_order_relaxed);
    double packet_age = -1.0;
    if (last_packet < 0) {
      status.summary(DiagnosticStatus::WARN, "waiting for valid RIP packets");
    } else {
      packet_age = static_cast<double>(steady_time_nanoseconds() - last_packet) / 1.0e9;
      if (packet_age > settings_.packet_stale_timeout) {
        status.summary(DiagnosticStatus::WARN, "RIP packet stream is stale");
      } else {
        status.summary(DiagnosticStatus::OK, "receiving RIP packets");
      }
    }
    status.add("last_packet_age_seconds", packet_age);
  }

  switch (configuration_state_.load()) {
    case ConfigurationState::DISABLED:
      status.add("configuration", "disabled");
      break;
    case ConfigurationState::PENDING:
      status.add("configuration", "pending");
      break;
    case ConfigurationState::SUCCESSFUL: {
        status.add("configuration", "successful");
        std::lock_guard<std::mutex> lock(configuration_mutex_);
        if (!configuration_unsupported_.empty()) {
          status.add("configuration_unsupported", join(configuration_unsupported_));
          status.mergeSummary(
            DiagnosticStatus::WARN, "sonar firmware lacks requested settings");
        }
        break;
      }
    case ConfigurationState::FAILED: {
        status.add("configuration", "failed");
        std::lock_guard<std::mutex> lock(configuration_mutex_);
        status.add("configuration_error", configuration_error_);
        status.mergeSummary(DiagnosticStatus::WARN, "sonar configuration failed");
        break;
      }
  }

  const auto offset_seconds =
    static_cast<double>(last_sensor_clock_offset_ns_.load(std::memory_order_relaxed)) / 1.0e9;
  if (!settings_.use_sensor_timestamps) {
    status.add("timestamp_source", "receive (sensor timestamps disabled)");
  } else {
    switch (timestamp_source_.load(std::memory_order_relaxed)) {
      case TimestampSource::NONE:
        status.add("timestamp_source", "none yet");
        break;
      case TimestampSource::SENSOR:
        status.add("timestamp_source", "sensor");
        status.add("sensor_clock_offset_seconds", offset_seconds);
        break;
      case TimestampSource::RECEIVE:
        status.add("timestamp_source", "receive-aligned (sensor clock unsynchronized)");
        status.add("sensor_clock_offset_seconds", offset_seconds);
        status.mergeSummary(
          DiagnosticStatus::WARN,
          "sonar clock unsynchronized; sensor timestamps aligned to receive time");
        break;
    }
  }
  status.add("sensor_clock_fallbacks", value(metrics_.sensor_clock_fallbacks));
  if (receiver_) {
    status.add("udp_receive_buffer_bytes", receiver_->receive_buffer_size());
  }
  status.add("datagrams", value(metrics_.datagrams));
  status.add("rejected_sources", value(metrics_.rejected_sources));
  status.add("valid_packets", value(metrics_.valid_packets));
  status.add("malformed_packets", value(metrics_.malformed_packets));
  status.add("crc_failures", value(metrics_.crc_failures));
  status.add("unknown_messages", value(metrics_.unknown_messages));
  status.add("range_images", value(metrics_.range_images));
  status.add("bitmap_images", value(metrics_.bitmap_images));
  status.add("imu_batches", value(metrics_.imu_batches));
  status.add("sequence_gaps", value(metrics_.sequence_gaps));
  status.add("duplicates", value(metrics_.duplicate_messages));
  status.add("out_of_order", value(metrics_.out_of_order_messages));
  status.add("sequence_resets", value(metrics_.sequence_resets));
  status.add("socket_errors", value(metrics_.socket_errors));
}

}  // namespace sonar3d

RCLCPP_COMPONENTS_REGISTER_NODE(sonar3d::SonarDriver)
