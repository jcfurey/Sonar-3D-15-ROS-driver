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
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace sonar3d
{
namespace
{

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

[[nodiscard]] std::string make_base_url(const std::string & address)
{
  if (address.empty() || address.find_first_of("/ \t\r\n") != std::string::npos) {
    throw std::invalid_argument("sonar_ip must be an IPv4 address, IPv6 address, or hostname");
  }
  if (address.front() == '[' && address.back() == ']') {
    return "http://" + address;
  }
  if (address.find(':') != std::string::npos) {
    return "http://[" + address + "]";
  }
  return "http://" + address;
}

[[nodiscard]] std::string json_number(double value)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return stream.str();
}

[[nodiscard]] bool stopping(const std::function<bool()> & stop_requested)
{
  return stop_requested && stop_requested();
}

std::size_t discard_response(char *, std::size_t size, std::size_t count, void *)
{
  return size * count;
}

}  // namespace

HttpSonarApi::HttpSonarApi(std::string address, std::chrono::milliseconds timeout)
: base_url_(make_base_url(address)), timeout_(timeout)
{
  if (timeout_.count() <= 0) {
    throw std::invalid_argument("HTTP timeout must be positive");
  }
  initialize_curl();
}

void HttpSonarApi::set_speed_of_sound(double meters_per_second)
{
  post_json(
    "/api/v1/integration/acoustics/speed_of_sound",
    json_number(meters_per_second),
    std::max(timeout_,
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::seconds(30))));
}

void HttpSonarApi::set_acoustics_enabled(bool enabled)
{
  post_json("/api/v1/integration/acoustics/enabled", enabled ? "true" : "false", timeout_);
}

void HttpSonarApi::set_udp_multicast()
{
  post_json(
    "/api/v1/integration/udp",
    R"({"mode":"multicast","unicast_destination_ip":"","unicast_destination_port":0})",
    timeout_);
}

void HttpSonarApi::post_json(
  const std::string & path,
  const std::string & body,
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

  std::array<char, CURL_ERROR_SIZE> error_buffer{};
  const auto url = base_url_ + path;
  curl_easy_setopt(handle.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(handle.get(), CURLOPT_POST, 1L);
  curl_easy_setopt(handle.get(), CURLOPT_POSTFIELDS, body.data());
  curl_easy_setopt(handle.get(), CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body.size()));
  curl_easy_setopt(handle.get(), CURLOPT_HTTPHEADER, headers.get());
  curl_easy_setopt(
    handle.get(), CURLOPT_CONNECTTIMEOUT_MS,
    static_cast<long>(timeout.count()));  // NOLINT(runtime/int): required by libcurl
  curl_easy_setopt(
    handle.get(), CURLOPT_TIMEOUT_MS,
    static_cast<long>(timeout.count()));  // NOLINT(runtime/int): required by libcurl
  curl_easy_setopt(handle.get(), CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(handle.get(), CURLOPT_TCP_KEEPALIVE, 1L);
  curl_easy_setopt(handle.get(), CURLOPT_WRITEFUNCTION, discard_response);
  curl_easy_setopt(handle.get(), CURLOPT_ERRORBUFFER, error_buffer.data());

  const auto result = curl_easy_perform(handle.get());
  if (result != CURLE_OK) {
    const auto detail = error_buffer.front() ==
      '\0' ? curl_easy_strerror(result) : error_buffer.data();
    throw std::runtime_error("HTTP POST " + path + " failed: " + detail);
  }

  long status_code{};  // NOLINT(runtime/int): required by libcurl
  curl_easy_getinfo(handle.get(), CURLINFO_RESPONSE_CODE, &status_code);
  if (status_code < 200 || status_code >= 300) {
    throw std::runtime_error(
      "HTTP POST " + path + " returned status " + std::to_string(status_code));
  }
}

void validate_configuration(double speed_of_sound, double timeout_seconds)
{
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0) {
    throw std::invalid_argument("http_timeout must be finite and greater than zero");
  }
  if (!std::isfinite(speed_of_sound) ||
    (speed_of_sound != 0.0 && (speed_of_sound < 1000.0 || speed_of_sound > 2000.0)))
  {
    throw std::invalid_argument("speed_of_sound must be zero or between 1000 and 2000 m/s");
  }
}

std::vector<std::string> configure_sonar(
  SonarApi & api,
  double speed_of_sound,
  const std::function<bool()> & stop_requested)
{
  validate_configuration(speed_of_sound, 1.0);
  std::vector<std::string> applied;

  if (speed_of_sound != 0.0 && !stopping(stop_requested)) {
    api.set_speed_of_sound(speed_of_sound);
    applied.emplace_back("speed_of_sound");
  }
  if (stopping(stop_requested)) {
    return applied;
  }
  api.set_acoustics_enabled(true);
  applied.emplace_back("acoustics");

  if (stopping(stop_requested)) {
    return applied;
  }
  api.set_udp_multicast();
  applied.emplace_back("multicast");
  return applied;
}

}  // namespace sonar3d
