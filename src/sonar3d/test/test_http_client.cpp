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

  double speed{};
  bool acoustics{};
  std::vector<std::string> calls;
};

TEST(HttpClient, AppliesDefaultConfigurationInOrder)
{
  FakeSonarApi api;

  const auto applied = sonar3d::configure_sonar(api);

  EXPECT_EQ(applied, (std::vector<std::string>{"acoustics", "multicast"}));
  EXPECT_EQ(api.calls, applied);
  EXPECT_TRUE(api.acoustics);
}

TEST(HttpClient, AppliesOptionalSpeedFirst)
{
  FakeSonarApi api;

  const auto applied = sonar3d::configure_sonar(api, 1491.0);

  EXPECT_EQ(
    applied,
    (std::vector<std::string>{"speed_of_sound", "acoustics", "multicast"}));
  EXPECT_EQ(api.calls, applied);
  EXPECT_DOUBLE_EQ(api.speed, 1491.0);
}

TEST(HttpClient, StopsCleanlyBetweenRequests)
{
  FakeSonarApi api;
  int checks{};
  const auto applied = sonar3d::configure_sonar(api, 1491.0, [&checks]() {
        return checks++ >= 1;
    });

  EXPECT_EQ(applied, (std::vector<std::string>{"speed_of_sound"}));
}

TEST(HttpClient, RejectsInvalidConfiguration)
{
  FakeSonarApi api;
  EXPECT_THROW(sonar3d::configure_sonar(api, 999.0), std::invalid_argument);
  EXPECT_THROW(sonar3d::configure_sonar(api, 2001.0), std::invalid_argument);
  EXPECT_THROW(sonar3d::validate_configuration(0.0, 0.0), std::invalid_argument);
}

}  // namespace
