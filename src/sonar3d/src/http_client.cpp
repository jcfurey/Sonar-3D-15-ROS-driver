// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include "sonar3d/http_client.hpp"

#include <curl/curl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace sonar3d
{
namespace
{

using nlohmann::json;

constexpr std::size_t kMaximumResponseSize = 1024U * 1024U;
// Reconfiguring the acoustics can take about 20 s on the sonar.
constexpr std::chrono::milliseconds kAcousticsTimeout{30'000};

struct CurlHandleDeleter
{
  void operator()(CURL * handle) const
  {
    curl_easy_cleanup(handle);
  }
};

struct CurlHeadersDeleter
{
  void operator()(curl_slist * headers) const
  {
    curl_slist_free_all(headers);
  }
};

struct Transfer
{
  std::string body;
  bool too_large{false};
  const std::function<bool()> * cancelled{nullptr};
};

void initialize_curl()
{
  static std::once_flag flag;
  std::call_once(flag, []() {
      const auto result = curl_global_init(CURL_GLOBAL_DEFAULT);
      if (result != CURLE_OK) {
        throw std::runtime_error("failed to initialize libcurl");
      }
  });
}

[[nodiscard]] std::string make_base_url(const std::string & address, std::uint16_t port)
{
  if (address.empty() || address.find_first_of("/ \t\r\n") != std::string::npos) {
    throw std::invalid_argument("sonar_ip must be an IPv4 address, IPv6 address, or hostname");
  }
  if (port == 0) {
    throw std::invalid_argument("HTTP port must be greater than zero");
  }
  auto host = address;
  if (address.find(':') != std::string::npos &&
    !(address.front() == '[' && address.back() == ']'))
  {
    host = "[" + address + "]";
  }
  return "http://" + host + ":" + std::to_string(port);
}

[[nodiscard]] bool stopping(const std::function<bool()> & stop_requested)
{
  return stop_requested && stop_requested();
}

std::size_t collect_response(char * data, std::size_t size, std::size_t count, void * user)
{
  auto & transfer = *static_cast<Transfer *>(user);
  const auto bytes = size * count;
  if (transfer.body.size() + bytes > kMaximumResponseSize) {
    transfer.too_large = true;
    return 0;
  }
  transfer.body.append(data, bytes);
  return bytes;
}

int report_progress(void * user, curl_off_t, curl_off_t, curl_off_t, curl_off_t)
{
  const auto & transfer = *static_cast<Transfer *>(user);
  return transfer.cancelled != nullptr && *transfer.cancelled && (*transfer.cancelled)() ? 1 : 0;
}

template<typename Parse>
[[nodiscard]] auto parse_json(const std::string & text, const char * what, Parse parse)
{
  try {
    return parse(json::parse(text));
  } catch (const json::exception & error) {
    throw std::runtime_error(std::string("malformed ") + what + " response: " + error.what());
  }
}

[[nodiscard]] TimeStatus time_status_from_json(const json & value)
{
  TimeStatus result;
  result.system_time = value.at("system_time").get<std::string>();
  result.ntp_synced = value.at("ntp_synced").get<bool>();
  result.ntp_synced_to = value.at("ntp_synced_to").get<std::string>();
  const auto & since = value.at("ntp_seconds_since_last_sync");
  if (!since.is_null()) {
    result.seconds_since_last_sync = since.get<std::int64_t>();
  }
  return result;
}

[[nodiscard]] std::chrono::milliseconds milliseconds(double seconds)
{
  return std::chrono::milliseconds(static_cast<std::int64_t>(std::llround(seconds * 1000.0)));
}

// Optional settings were added in later sonar releases, which answer 404
// before then. Imaging still works, so report them instead of failing.
template<typename Apply>
void apply_optional(ConfigurationResult & result, const char * name, Apply apply)
{
  try {
    apply();
    result.applied.emplace_back(name);
  } catch (const HttpStatusError & error) {
    if (error.status() != 404) {
      throw;
    }
    result.unsupported.emplace_back(name);
  }
}

}  // namespace

DeviceInfo parse_device_info(const std::string & text)
{
  return parse_json(text, "about", [](const json & value) {
             DeviceInfo result;
             result.product_name = value.at("product_name").get<std::string>();
             result.version = value.at("version").get<std::string>();
             result.version_short = value.at("version_short").get<std::string>();
             result.chip_id = value.at("chipid").get<std::string>();
             result.variant = value.at("variant").get<std::string>();
             result.product_id = value.at("product_id").get<std::int64_t>();
             result.hardware_revision = value.at("hardware_revision").get<std::int64_t>();
             result.ready = value.at("is_ready").get<bool>();
             return result;
  });
}

std::map<std::string, StatusEntry> parse_status(const std::string & text)
{
  return parse_json(text, "status", [](const json & value) {
             std::map<std::string, StatusEntry> result;
             for (const auto & [component, entry] : value.items()) {
               if (entry.is_null()) {
                 continue;
               }
               result[component] = StatusEntry{
                 entry.at("id").get<std::string>(),
                 entry.at("message").get<std::string>(),
                 entry.at("status").get<std::string>(),
                 entry.at("operational").get<bool>(),
               };
             }
             return result;
  });
}

double parse_temperature(const std::string & text)
{
  return parse_json(text, "temperature", [](const json & value) {
             return value.get<double>();
  });
}

TimeStatus parse_time_status(const std::string & text)
{
  return parse_json(text, "time status", time_status_from_json);
}

NtpSyncResult parse_ntp_sync(const std::string & text)
{
  return parse_json(text, "NTP sync", [](const json & value) {
             return NtpSyncResult{
             value.at("success").get<bool>(),
             value.at("message").get<std::string>(),
             time_status_from_json(value.at("status")),
             };
  });
}

HttpSonarApi::HttpSonarApi(
  std::string address, std::chrono::milliseconds timeout,
  std::function<bool()> cancelled, std::uint16_t port)
: base_url_(make_base_url(address, port)), timeout_(timeout), cancelled_(std::move(cancelled))
{
  if (timeout_.count() <= 0) {
    throw std::invalid_argument("HTTP timeout must be positive");
  }
  initialize_curl();
}

void HttpSonarApi::set_speed_of_sound(double meters_per_second)
{
  request(
    "/api/v1/integration/acoustics/speed_of_sound", json(meters_per_second).dump(),
    std::max(timeout_, kAcousticsTimeout));
}

void HttpSonarApi::set_salinity(const std::string & salinity)
{
  request(
    "/api/v1/integration/acoustics/salinity", json(salinity).dump(),
    std::max(timeout_, kAcousticsTimeout));
}

void HttpSonarApi::set_acoustics_mode(const std::string & mode)
{
  request(
    "/api/v1/integration/acoustics/mode", json(mode).dump(),
    std::max(timeout_, kAcousticsTimeout));
}

void HttpSonarApi::set_range(double minimum_meters, double maximum_meters)
{
  request(
    "/api/v1/integration/acoustics/range",
    json{{"min", minimum_meters}, {"max", maximum_meters}}.dump(),
    std::max(timeout_, kAcousticsTimeout));
}

void HttpSonarApi::set_acoustics_enabled(bool enabled)
{
  request("/api/v1/integration/acoustics/enabled", json(enabled).dump(), timeout_);
}

void HttpSonarApi::set_udp_output(const UdpOutput & output)
{
  const bool unicast = output.mode == UdpOutputMode::UNICAST;
  request(
    "/api/v1/integration/udp",
    json{
      {"mode", unicast ? "unicast" : "multicast"},
      {"unicast_destination_ip", unicast ? output.unicast_destination_ip : ""},
      {"unicast_destination_port", unicast ? output.unicast_destination_port : 0},
    }.dump(),
    timeout_);
}

void HttpSonarApi::set_imu_output_enabled(bool enabled)
{
  request("/api/v1/integration/output/imu-batch/enabled", json(enabled).dump(), timeout_);
}

void HttpSonarApi::set_ntp_server(const std::string & address)
{
  request("/api/v1/integration/time/ntp", json{{"ntp_address", address}}.dump(), timeout_);
}

NtpSyncResult HttpSonarApi::force_ntp_sync(double timeout_seconds)
{
  return parse_ntp_sync(
    request(
      "/api/v1/integration/time/ntp/force-sync",
      json{{"timeout_seconds", timeout_seconds}}.dump(),
      timeout_ + milliseconds(timeout_seconds)));
}

DeviceInfo HttpSonarApi::about() const
{
  return parse_device_info(request("/api/v1/integration/about", std::nullopt, timeout_));
}

std::map<std::string, StatusEntry> HttpSonarApi::status() const
{
  return parse_status(request("/api/v1/integration/status", std::nullopt, timeout_));
}

double HttpSonarApi::temperature() const
{
  return parse_temperature(request("/api/v1/integration/temperature", std::nullopt, timeout_));
}

TimeStatus HttpSonarApi::time_status() const
{
  return parse_time_status(request("/api/v1/integration/time/status", std::nullopt, timeout_));
}

std::string HttpSonarApi::request(
  const std::string & path,
  const std::optional<std::string> & post_body,
  std::chrono::milliseconds timeout) const
{
  std::unique_ptr<CURL, CurlHandleDeleter> handle(curl_easy_init());
  if (!handle) {
    throw std::runtime_error("failed to create an HTTP request");
  }

  std::unique_ptr<curl_slist, CurlHeadersDeleter> headers(
    curl_slist_append(nullptr, "Content-Type: application/json"));
  if (!headers) {
    throw std::runtime_error("failed to allocate HTTP headers");
  }

  const auto method = post_body ? std::string("POST ") : std::string("GET ");
  Transfer transfer;
  transfer.cancelled = &cancelled_;
  std::array<char, CURL_ERROR_SIZE> error_buffer{};
  const auto url = base_url_ + path;
  curl_easy_setopt(handle.get(), CURLOPT_URL, url.c_str());
  if (post_body) {
    curl_easy_setopt(handle.get(), CURLOPT_POST, 1L);
    curl_easy_setopt(handle.get(), CURLOPT_POSTFIELDS, post_body->data());
    curl_easy_setopt(
      handle.get(), CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(post_body->size()));
    curl_easy_setopt(handle.get(), CURLOPT_HTTPHEADER, headers.get());
  }
  curl_easy_setopt(
    handle.get(), CURLOPT_CONNECTTIMEOUT_MS,
    static_cast<long>(timeout.count()));  // NOLINT(runtime/int): required by libcurl
  curl_easy_setopt(
    handle.get(), CURLOPT_TIMEOUT_MS,
    static_cast<long>(timeout.count()));  // NOLINT(runtime/int): required by libcurl
  curl_easy_setopt(handle.get(), CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(handle.get(), CURLOPT_TCP_KEEPALIVE, 1L);
  curl_easy_setopt(handle.get(), CURLOPT_WRITEFUNCTION, collect_response);
  curl_easy_setopt(handle.get(), CURLOPT_WRITEDATA, &transfer);
  curl_easy_setopt(handle.get(), CURLOPT_NOPROGRESS, 0L);
  curl_easy_setopt(handle.get(), CURLOPT_XFERINFOFUNCTION, report_progress);
  curl_easy_setopt(handle.get(), CURLOPT_XFERINFODATA, &transfer);
  curl_easy_setopt(handle.get(), CURLOPT_ERRORBUFFER, error_buffer.data());

  const auto result = curl_easy_perform(handle.get());
  if (result == CURLE_ABORTED_BY_CALLBACK) {
    throw std::runtime_error("HTTP " + method + path + " was cancelled");
  }
  if (transfer.too_large) {
    throw std::runtime_error("HTTP " + method + path + " response exceeds 1 MiB");
  }
  if (result != CURLE_OK) {
    const auto detail = error_buffer.front() ==
      '\0' ? curl_easy_strerror(result) : error_buffer.data();
    throw std::runtime_error("HTTP " + method + path + " failed: " + detail);
  }

  long status_code{};  // NOLINT(runtime/int): required by libcurl
  curl_easy_getinfo(handle.get(), CURLINFO_RESPONSE_CODE, &status_code);
  if (status_code < 200 || status_code >= 300) {
    throw HttpStatusError(
      "HTTP " + method + path + " returned status " + std::to_string(status_code), status_code);
  }
  return std::move(transfer.body);
}

void validate_configuration(const ConfigurationRequest & request, double timeout_seconds)
{
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0) {
    throw std::invalid_argument("http_timeout must be finite and greater than zero");
  }
  const auto speed = request.speed_of_sound;
  if (!std::isfinite(speed) || (speed > 0.0 && (speed < 1000.0 || speed > 2000.0))) {
    throw std::invalid_argument(
            "speed_of_sound must be negative (keep), zero (automatic), or 1000-2000 m/s");
  }
  if (!request.salinity.empty() && request.salinity != "fresh" && request.salinity != "salt") {
    throw std::invalid_argument("salinity must be empty, \"fresh\" or \"salt\"");
  }
  if (!request.acoustics_mode.empty() && request.acoustics_mode != "low-frequency" &&
    request.acoustics_mode != "high-frequency")
  {
    throw std::invalid_argument(
            "acoustics_mode must be empty, \"low-frequency\" or \"high-frequency\"");
  }
  const bool range_set = request.range_max > 0.0;
  if (!std::isfinite(request.range_min) || !std::isfinite(request.range_max) ||
    request.range_max < 0.0 ||
    (range_set && (request.range_min < 0.0 || request.range_min >= request.range_max)))
  {
    throw std::invalid_argument(
            "range_max must be zero (keep) or greater than range_min, which must not be negative");
  }
  if (request.udp_output.mode == UdpOutputMode::UNICAST &&
    (request.udp_output.unicast_destination_ip.empty() ||
    request.udp_output.unicast_destination_ip == "0.0.0.0" ||
    request.udp_output.unicast_destination_port == 0))
  {
    throw std::invalid_argument(
            "unicast output needs this computer's address and port; set interface_address");
  }
  if (!std::isfinite(request.ntp_sync_timeout) || request.ntp_sync_timeout < 0.0) {
    throw std::invalid_argument("ntp_sync_timeout must be zero or a positive number of seconds");
  }
}

ConfigurationResult configure_sonar(
  SonarApi & api,
  const ConfigurationRequest & request,
  const std::function<bool()> & stop_requested)
{
  validate_configuration(request, 1.0);
  ConfigurationResult result;
  const auto step = [&stop_requested](bool wanted) {
      return wanted && !stopping(stop_requested);
    };

  if (step(!request.ntp_server.empty())) {
    apply_optional(result, "ntp_server", [&] {api.set_ntp_server(request.ntp_server);});
  }
  // Automatic speed of sound depends on salinity, so set salinity first.
  if (step(!request.salinity.empty())) {
    apply_optional(result, "salinity", [&] {api.set_salinity(request.salinity);});
  }
  if (step(request.speed_of_sound >= 0.0)) {
    api.set_speed_of_sound(request.speed_of_sound);
    result.applied.emplace_back("speed_of_sound");
  }
  if (step(!request.acoustics_mode.empty())) {
    apply_optional(result, "acoustics_mode", [&] {api.set_acoustics_mode(request.acoustics_mode);});
  }
  if (step(request.range_max > 0.0)) {
    api.set_range(request.range_min, request.range_max);
    result.applied.emplace_back("range");
  }
  if (step(true)) {
    api.set_acoustics_enabled(true);
    result.applied.emplace_back("acoustics");
  }
  if (step(true)) {
    api.set_udp_output(request.udp_output);
    result.applied.emplace_back(
      request.udp_output.mode == UdpOutputMode::UNICAST ? "unicast" : "multicast");
  }
  if (step(request.imu_output)) {
    apply_optional(result, "imu_output", [&] {api.set_imu_output_enabled(true);});
  }
  if (step(request.ntp_sync_timeout > 0.0)) {
    try {
      apply_optional(
        result, "ntp_sync", [&] {result.ntp_sync = api.force_ntp_sync(request.ntp_sync_timeout);});
    } catch (const HttpStatusError & error) {
      // 409: a sync is already running, which is what was asked for.
      if (error.status() != 409) {
        throw;
      }
      result.ntp_sync = NtpSyncResult{false, "an NTP sync is already in progress", {}};
    }
  }
  return result;
}

}  // namespace sonar3d
