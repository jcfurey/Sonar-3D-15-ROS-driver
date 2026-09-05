// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

#include "sonar3d/playback_clock.hpp"

namespace
{

using namespace std::chrono_literals;
using Clock = sonar3d::PlaybackClock::Clock;
constexpr std::int64_t kStamp = 1'700'000'000'000'000'000;
const Clock::time_point kStart{10s};

TEST(PlaybackClock, ProcessingTimeDoesNotAccumulateAcrossFrames)
{
  sonar3d::PlaybackClock clock(1.0);
  EXPECT_EQ(clock.deadline(kStamp, kStart), kStart);
  for (int frame = 1; frame <= 100; ++frame) {
    const auto target = kStart + frame * 50ms;
    EXPECT_EQ(clock.deadline(kStamp + frame * 50'000'000LL, target - 43ms), target);
  }
}

TEST(PlaybackClock, InterleavedOlderImuAndRepeatedImageTimesKeepTheSameAnchor)
{
  sonar3d::PlaybackClock clock(1.0);
  EXPECT_EQ(clock.deadline(kStamp, kStart), kStart);
  for (int frame = 0; frame < 100; ++frame) {
    const auto stamp = kStamp + frame * 50'000'000LL;
    const auto now = kStart + frame * 50ms;
    EXPECT_EQ(clock.deadline(stamp, now), now);
    EXPECT_EQ(clock.deadline(stamp, now + 1ms), now + 1ms);
    EXPECT_EQ(clock.deadline(stamp - 40'000'000, now + 2ms), now + 2ms);
    EXPECT_EQ(clock.deadline(stamp + 50'000'000, now + 3ms), now + 50ms);
  }
}

TEST(PlaybackClock, MissingTimestampsAndLateProcessingDoNotResetPacing)
{
  sonar3d::PlaybackClock clock(1.0);
  EXPECT_EQ(clock.deadline(std::nullopt, kStart), kStart);
  EXPECT_EQ(clock.deadline(kStamp, kStart + 1s), kStart + 1s);
  EXPECT_EQ(clock.deadline(std::nullopt, kStart + 1020ms), kStart + 1020ms);
  EXPECT_EQ(clock.deadline(kStamp + 50'000'000, kStart + 1080ms), kStart + 1080ms);
  EXPECT_EQ(clock.deadline(kStamp + 100'000'000, kStart + 1081ms), kStart + 1100ms);
}

TEST(PlaybackClock, ScalesBothFasterAndSlowerPlayback)
{
  for (const auto factor : {0.5, 1.0, 2.0}) {
    sonar3d::PlaybackClock clock(factor);
    EXPECT_EQ(clock.deadline(kStamp, kStart), kStart);
    const auto expected = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / factor));
    EXPECT_EQ(clock.deadline(kStamp + 1'000'000'000, kStart + 10ms), kStart + expected);
  }
}

TEST(PlaybackClock, ReanchorsAfterARecordedClockReset)
{
  sonar3d::PlaybackClock clock(1.0);
  EXPECT_EQ(clock.deadline(kStamp, kStart), kStart);
  EXPECT_EQ(clock.deadline(kStamp + 2'000'000'000, kStart + 2s), kStart + 2s);
  EXPECT_EQ(clock.deadline(kStamp, kStart + 2010ms), kStart + 2010ms);
  EXPECT_EQ(clock.deadline(kStamp + 50'000'000, kStart + 2011ms), kStart + 2060ms);
}

TEST(PlaybackClock, BoundsUnrepresentableDeadlinesAndRejectsInvalidRates)
{
  sonar3d::PlaybackClock clock(std::numeric_limits<double>::min());
  EXPECT_EQ(clock.deadline(kStamp, kStart), kStart);
  EXPECT_EQ(clock.deadline(kStamp + 1, kStart), Clock::time_point::max());
  EXPECT_THROW(sonar3d::PlaybackClock(0.0), std::invalid_argument);
  EXPECT_THROW(sonar3d::PlaybackClock(-1.0), std::invalid_argument);
  EXPECT_THROW(
    sonar3d::PlaybackClock(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(
    sonar3d::PlaybackClock(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

}  // namespace
