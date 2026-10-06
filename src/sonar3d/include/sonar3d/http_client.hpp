// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

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

class SonarApi
{
public:
  virtual ~SonarApi() = default;

  virtual void set_speed_of_sound(double meters_per_second) = 0;
  virtual void set_acoustics_enabled(bool enabled) = 0;
  virtual void set_udp_multicast() = 0;
  // Public ImuBatch output; available from sonar release 1.8.0.
  virtual void set_imu_output_enabled(bool enabled) = 0;
};

class HttpSonarApi final : public SonarApi
{
public:
  HttpSonarApi(std::string address, std::chrono::milliseconds timeout);

  void set_speed_of_sound(double meters_per_second) override;
  void set_acoustics_enabled(bool enabled) override;
  void set_udp_multicast() override;
  void set_imu_output_enabled(bool enabled) override;

private:
  void post_json(
    const std::string & path,
    const std::string & body,
    std::chrono::milliseconds timeout) const;

  std::string base_url_;
  std::chrono::milliseconds timeout_;
};

struct ConfigurationRequest
{
  // Zero keeps the device setting.
  double speed_of_sound{0.0};
  bool imu_output{false};
};

struct ConfigurationResult
{
  std::vector<std::string> applied;
  // Optional settings the sonar firmware does not provide.
  std::vector<std::string> unsupported;
};

void validate_configuration(double speed_of_sound, double timeout_seconds);

// Applies speed of sound, acoustics, UDP multicast, then IMU output, stopping
// between requests when stop_requested returns true.
[[nodiscard]] ConfigurationResult configure_sonar(
  SonarApi & api,
  const ConfigurationRequest & request = {},
  const std::function<bool()> & stop_requested = {});

}  // namespace sonar3d
