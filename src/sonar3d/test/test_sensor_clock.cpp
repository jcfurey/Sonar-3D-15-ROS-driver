// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

#include "sonar3d/sensor_clock.hpp"

namespace
{

constexpr std::int64_t kMillisecond = 1'000'000;
constexpr std::int64_t kSecond = 1'000'000'000;

TEST(SensorClockOffset, KeepsTheSmallestDelayAndIgnoresLatencySpikes)
{
  sonar3d::SensorClockOffset clock(10 * kSecond, kSecond);
  const std::int64_t offset = 500 * kSecond;
  EXPECT_EQ(clock.update(offset + 40 * kMillisecond, 0), offset + 40 * kMillisecond);
  EXPECT_EQ(clock.update(offset + 5 * kMillisecond, kSecond), offset + 5 * kMillisecond);
  EXPECT_EQ(clock.update(offset + 300 * kMillisecond, 2 * kSecond), offset + 5 * kMillisecond);
  EXPECT_EQ(clock.update(offset + 8 * kMillisecond, 3 * kSecond), offset + 5 * kMillisecond);
}

TEST(SensorClockOffset, FollowsDriftAfterTwoWindows)
{
  sonar3d::SensorClockOffset clock(10 * kSecond, kSecond);
  EXPECT_EQ(clock.update(100 * kMillisecond, 0), 100 * kMillisecond);
  // The first rollover still remembers the previous window's minimum.
  EXPECT_EQ(clock.update(102 * kMillisecond, 10 * kSecond), 100 * kMillisecond);
  EXPECT_EQ(clock.update(103 * kMillisecond, 15 * kSecond), 100 * kMillisecond);
  // After the second rollover only drifted observations remain.
  EXPECT_EQ(clock.update(104 * kMillisecond, 20 * kSecond), 102 * kMillisecond);
  EXPECT_EQ(clock.update(105 * kMillisecond, 30 * kSecond), 104 * kMillisecond);
}

TEST(SensorClockOffset, RestartsWhenTheSonarClockSteps)
{
  sonar3d::SensorClockOffset clock(10 * kSecond, kSecond);
  EXPECT_EQ(clock.update(1'000 * kSecond, 0), 1'000 * kSecond);
  // NTP moves the sonar clock forward: a smaller offset is accepted at once.
  EXPECT_EQ(clock.update(20 * kMillisecond, kSecond), 20 * kMillisecond);
  // A backward step beyond the threshold restarts instead of being ignored,
  // and later faster deliveries still lower the new estimate.
  EXPECT_EQ(clock.update(5 * kSecond, 2 * kSecond), 5 * kSecond);
  EXPECT_EQ(clock.update(4 * kSecond, 3 * kSecond), 4 * kSecond);
  // A spike within the threshold is still ignored.
  EXPECT_EQ(clock.update(4 * kSecond + 900 * kMillisecond, 4 * kSecond), 4 * kSecond);

  clock.reset();
  EXPECT_EQ(clock.update(7 * kSecond, 5 * kSecond), 7 * kSecond);
}

TEST(SensorClockOffset, RejectsNonPositiveSettings)
{
  EXPECT_THROW(sonar3d::SensorClockOffset(0, kSecond), std::invalid_argument);
  EXPECT_THROW(sonar3d::SensorClockOffset(kSecond, 0), std::invalid_argument);
}

}  // namespace
