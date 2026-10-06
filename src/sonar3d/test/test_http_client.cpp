// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

#include "sonar3d/http_client.hpp"

namespace
{

class FakeSonarApi final : public sonar3d::SonarApi
{
public:
  void set_speed_of_sound(double value) override
  {
    speed = value;
    calls.emplace_back("speed_of_sound");
  }

  void set_acoustics_enabled(bool value) override
  {
    acoustics = value;
    calls.emplace_back("acoustics");
  }

  void set_udp_multicast() override
  {
    calls.emplace_back("multicast");
  }

  void set_imu_output_enabled(bool value) override
  {
    if (imu_status != 0) {
      throw sonar3d::HttpStatusError("imu-batch", imu_status);
    }
    imu_output = value;
    calls.emplace_back("imu_output");
  }

  double speed{};
  bool acoustics{};
  bool imu_output{};
  long imu_status{};  // NOLINT(runtime/int)
  std::vector<std::string> calls;
};

TEST(HttpClient, AppliesDefaultConfigurationInOrder)
{
  FakeSonarApi api;

  const auto result = sonar3d::configure_sonar(api);

  EXPECT_EQ(result.applied, (std::vector<std::string>{"acoustics", "multicast"}));
  EXPECT_EQ(api.calls, result.applied);
  EXPECT_TRUE(result.unsupported.empty());
  EXPECT_TRUE(api.acoustics);
  EXPECT_FALSE(api.imu_output);
}

TEST(HttpClient, AppliesOptionalSpeedFirstAndImuOutputLast)
{
  FakeSonarApi api;

  const auto result = sonar3d::configure_sonar(api, {1491.0, true});

  EXPECT_EQ(
    result.applied,
    (std::vector<std::string>{"speed_of_sound", "acoustics", "multicast", "imu_output"}));
  EXPECT_EQ(api.calls, result.applied);
  EXPECT_DOUBLE_EQ(api.speed, 1491.0);
  EXPECT_TRUE(api.imu_output);
}

TEST(HttpClient, ReportsImuOutputMissingFromOlderFirmware)
{
  FakeSonarApi api;
  api.imu_status = 404;

  const auto result = sonar3d::configure_sonar(api, {0.0, true});

  EXPECT_EQ(result.applied, (std::vector<std::string>{"acoustics", "multicast"}));
  EXPECT_EQ(result.unsupported, (std::vector<std::string>{"imu_output"}));

  api.imu_status = 500;
  EXPECT_THROW(
    static_cast<void>(sonar3d::configure_sonar(api, {0.0, true})), sonar3d::HttpStatusError);
}

TEST(HttpClient, StopsCleanlyBetweenRequests)
{
  FakeSonarApi api;
  int checks{};
  const auto result = sonar3d::configure_sonar(api, {1491.0, true}, [&checks]() {
        return checks++ >= 1;
    });

  EXPECT_EQ(result.applied, (std::vector<std::string>{"speed_of_sound"}));
}

TEST(HttpClient, RejectsInvalidConfiguration)
{
  FakeSonarApi api;
  EXPECT_THROW(
    static_cast<void>(sonar3d::configure_sonar(api, {999.0, false})), std::invalid_argument);
  EXPECT_THROW(
    static_cast<void>(sonar3d::configure_sonar(api, {2001.0, false})), std::invalid_argument);
  EXPECT_THROW(sonar3d::validate_configuration(0.0, 0.0), std::invalid_argument);
}

}  // namespace
