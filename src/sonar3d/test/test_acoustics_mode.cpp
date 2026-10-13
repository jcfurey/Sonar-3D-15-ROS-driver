// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>

#include "sonar3d/acoustics_mode.hpp"

namespace
{

using sonar3d::AcousticsMode;

TEST(AcousticsMode, FieldOfViewIdentifiesEachDocumentedMode)
{
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(90.0F, 40.0F), AcousticsMode::LOW_FREQUENCY);
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(40.0F, 40.0F), AcousticsMode::HIGH_FREQUENCY);
  // Small differences in the reported floats keep the mode.
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(89.5F, 40.4F), AcousticsMode::LOW_FREQUENCY);
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(40.6F, 39.5F), AcousticsMode::HIGH_FREQUENCY);
}

TEST(AcousticsMode, OtherFieldsOfViewAreUnknown)
{
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(60.0F, 40.0F), AcousticsMode::UNKNOWN);
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(90.0F, 20.0F), AcousticsMode::UNKNOWN);
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(40.0F, 90.0F), AcousticsMode::UNKNOWN);
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(0.0F, 0.0F), AcousticsMode::UNKNOWN);
  EXPECT_EQ(sonar3d::acoustics_mode_from_fov(nan, 40.0F), AcousticsMode::UNKNOWN);
}

TEST(AcousticsMode, NamesMatchTheHttpApi)
{
  for (const auto mode : {AcousticsMode::LOW_FREQUENCY, AcousticsMode::HIGH_FREQUENCY}) {
    EXPECT_EQ(sonar3d::parse_acoustics_mode(sonar3d::to_string(mode)), mode);
  }
  EXPECT_EQ(sonar3d::to_string(AcousticsMode::HIGH_FREQUENCY), "high-frequency");
  EXPECT_FALSE(sonar3d::parse_acoustics_mode(""));
  EXPECT_FALSE(sonar3d::parse_acoustics_mode("unknown"));
}

TEST(AcousticsMode, ExpectedRatesSeparateTheModes)
{
  const auto low = sonar3d::expected_image_rate(AcousticsMode::LOW_FREQUENCY);
  const auto high = sonar3d::expected_image_rate(AcousticsMode::HIGH_FREQUENCY);
  const auto unknown = sonar3d::expected_image_rate(AcousticsMode::UNKNOWN);
  // The measured 6 Hz low-frequency rate is accepted.
  EXPECT_LE(low.minimum_hz, 6.0);
  EXPECT_GE(low.maximum_hz, 6.0);
  EXPECT_LE(high.minimum_hz, 20.0);
  EXPECT_GE(high.maximum_hz, 20.0);
  // With diagnostic_updater's 10% tolerance, a high-frequency sonar imaging
  // at the low-frequency rate is reported, and vice versa.
  EXPECT_LT(low.maximum_hz * 1.1, high.minimum_hz * 0.9);
  EXPECT_EQ(unknown.minimum_hz, low.minimum_hz);
  EXPECT_EQ(unknown.maximum_hz, high.maximum_hz);
}

}  // namespace
