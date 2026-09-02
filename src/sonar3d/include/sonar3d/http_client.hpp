// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace sonar3d
{

class SonarApi
{
public:
  virtual ~SonarApi() = default;

  virtual void set_speed_of_sound(double meters_per_second) = 0;
  virtual void set_acoustics_enabled(bool enabled) = 0;
  virtual void set_udp_multicast() = 0;
};

class HttpSonarApi final : public SonarApi
{
public:
  HttpSonarApi(std::string address, std::chrono::milliseconds timeout);

  void set_speed_of_sound(double meters_per_second) override;
  void set_acoustics_enabled(bool enabled) override;
  void set_udp_multicast() override;

private:
  void post_json(
    const std::string & path,
    const std::string & body,
    std::chrono::milliseconds timeout) const;

  std::string base_url_;
  std::chrono::milliseconds timeout_;
};

void validate_configuration(double speed_of_sound, double timeout_seconds);

[[nodiscard]] std::vector<std::string> configure_sonar(
  SonarApi & api,
  double speed_of_sound = 0.0,
  const std::function<bool()> & stop_requested = {});

}  // namespace sonar3d
