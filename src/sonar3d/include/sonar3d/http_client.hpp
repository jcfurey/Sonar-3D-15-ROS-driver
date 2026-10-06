// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Client for the Sonar 3D-15 integration HTTP API, following Water Linked's
// swagger specification and the wlsonar 0.5.5 reference client.
namespace sonar3d
{

// A completed HTTP exchange whose status was not 2xx.
class HttpStatusError : public std::runtime_error
{
public:
  HttpStatusError(const std::string & message, long status)  // NOLINT(runtime/int)
  : std::runtime_error(message), status_(status) {}

  [[nodiscard]] long status() const noexcept {return status_;}  // NOLINT(runtime/int)

private:
  long status_;  // NOLINT(runtime/int): libcurl's response code type
};

enum class UdpOutputMode
{
  MULTICAST,
  UNICAST,
};

struct UdpOutput
{
  UdpOutputMode mode{UdpOutputMode::MULTICAST};
  // Unicast only: where the sonar sends RIP packets.
  std::string unicast_destination_ip;
  std::uint16_t unicast_destination_port{0};
};

// GET /about.
struct DeviceInfo
{
  std::string product_name;
  std::string version;
  std::string version_short;
  std::string chip_id;
  std::string variant;
  std::int64_t product_id{};
  std::int64_t hardware_revision{};
  bool ready{};
};

// One component of GET /status (sonar release 1.7.0 or newer).
struct StatusEntry
{
  std::string id;
  std::string message;
  // "ok", "warning" or "error".
  std::string status;
  bool operational{};
};

// GET /time/status (sonar release 1.7.1 or newer).
struct TimeStatus
{
  std::string system_time;
  bool ntp_synced{};
  std::string ntp_synced_to;
  std::optional<std::int64_t> seconds_since_last_sync;
};

// POST /time/ntp/force-sync.
struct NtpSyncResult
{
  bool success{};
  std::string message;
  TimeStatus status;
};

// Response parsers; each throws std::runtime_error for malformed JSON.
[[nodiscard]] DeviceInfo parse_device_info(const std::string & json);
// Keyed by component: api, temperature, systems_check and (1.7.1+) time.
[[nodiscard]] std::map<std::string, StatusEntry> parse_status(const std::string & json);
[[nodiscard]] double parse_temperature(const std::string & json);
[[nodiscard]] TimeStatus parse_time_status(const std::string & json);
[[nodiscard]] NtpSyncResult parse_ntp_sync(const std::string & json);

class SonarApi
{
public:
  virtual ~SonarApi() = default;

  // Zero selects automatic calculation from salinity and water temperature.
  virtual void set_speed_of_sound(double meters_per_second) = 0;
  // "fresh" or "salt" (1.7.0+).
  virtual void set_salinity(const std::string & salinity) = 0;
  // "low-frequency" or "high-frequency" (1.7.0+).
  virtual void set_acoustics_mode(const std::string & mode) = 0;
  virtual void set_range(double minimum_meters, double maximum_meters) = 0;
  virtual void set_acoustics_enabled(bool enabled) = 0;
  virtual void set_udp_output(const UdpOutput & output) = 0;
  // Public ImuBatch output (1.8.0+).
  virtual void set_imu_output_enabled(bool enabled) = 0;
  // An address or "auto" (1.7.1+).
  virtual void set_ntp_server(const std::string & address) = 0;
  virtual NtpSyncResult force_ntp_sync(double timeout_seconds) = 0;
};

class HttpSonarApi final : public SonarApi
{
public:
  // cancelled, when set, aborts an in-flight request soon after it returns true.
  HttpSonarApi(
    std::string address, std::chrono::milliseconds timeout,
    std::function<bool()> cancelled = {}, std::uint16_t port = 80);

  void set_speed_of_sound(double meters_per_second) override;
  void set_salinity(const std::string & salinity) override;
  void set_acoustics_mode(const std::string & mode) override;
  void set_range(double minimum_meters, double maximum_meters) override;
  void set_acoustics_enabled(bool enabled) override;
  void set_udp_output(const UdpOutput & output) override;
  void set_imu_output_enabled(bool enabled) override;
  void set_ntp_server(const std::string & address) override;
  NtpSyncResult force_ntp_sync(double timeout_seconds) override;

  [[nodiscard]] DeviceInfo about() const;
  [[nodiscard]] std::map<std::string, StatusEntry> status() const;
  [[nodiscard]] double temperature() const;
  [[nodiscard]] TimeStatus time_status() const;

private:
  // Returns the response body; throws HttpStatusError for non-2xx replies.
  std::string request(
    const std::string & path, const std::optional<std::string> & post_body,
    std::chrono::milliseconds timeout) const;

  std::string base_url_;
  std::chrono::milliseconds timeout_;
  std::function<bool()> cancelled_;
};

struct ConfigurationRequest
{
  // Negative keeps the device setting; zero selects automatic calculation
  // from salinity and water temperature; otherwise 1000-2000 m/s.
  double speed_of_sound{-1.0};
  // Empty keeps the device setting.
  std::string salinity;
  std::string acoustics_mode;
  // A zero maximum keeps the device's range.
  double range_min{0.0};
  double range_max{0.0};
  UdpOutput udp_output;
  bool imu_output{false};
  // Empty keeps the device setting.
  std::string ntp_server;
  // Positive: force an NTP sync and wait up to this many seconds for it.
  double ntp_sync_timeout{0.0};
};

struct ConfigurationResult
{
  std::vector<std::string> applied;
  // Requested settings the sonar firmware does not provide.
  std::vector<std::string> unsupported;
  std::optional<NtpSyncResult> ntp_sync;
};

void validate_configuration(const ConfigurationRequest & request, double timeout_seconds);

// Applies NTP server, salinity, speed of sound, acoustics mode, range,
// acoustics, UDP output, IMU output, then an NTP sync, stopping between
// requests when stop_requested returns true. Optional settings answered with
// 404 by older firmware are reported as unsupported instead of failing.
[[nodiscard]] ConfigurationResult configure_sonar(
  SonarApi & api,
  const ConfigurationRequest & request = {},
  const std::function<bool()> & stop_requested = {});

}  // namespace sonar3d
