// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <optional>
#include <string_view>

namespace sonar3d
{

// Water Linked's two acoustics modes (sonar release 1.7.0+). Both image
// 256 x 64 pixels; they differ in field of view and image rate.
enum class AcousticsMode : int
{
  UNKNOWN,
  LOW_FREQUENCY,
  HIGH_FREQUENCY,
};

// The HTTP API's names, "low-frequency" and "high-frequency".
[[nodiscard]] inline std::string_view to_string(AcousticsMode mode)
{
  switch (mode) {
    case AcousticsMode::LOW_FREQUENCY:
      return "low-frequency";
    case AcousticsMode::HIGH_FREQUENCY:
      return "high-frequency";
    case AcousticsMode::UNKNOWN:
      break;
  }
  return "unknown";
}

[[nodiscard]] inline std::optional<AcousticsMode> parse_acoustics_mode(std::string_view name)
{
  if (name == "low-frequency") {
    return AcousticsMode::LOW_FREQUENCY;
  }
  if (name == "high-frequency") {
    return AcousticsMode::HIGH_FREQUENCY;
  }
  return std::nullopt;
}

// Images carry no mode field, but each mode has its own documented field of
// view: 90 x 40 degrees in low frequency and 40 x 40 degrees in high frequency.
[[nodiscard]] inline AcousticsMode acoustics_mode_from_fov(
  float horizontal_degrees, float vertical_degrees)
{
  constexpr float kTolerance = 1.0F;
  const auto near = [](float value, float expected) {
      return std::isfinite(value) && std::fabs(value - expected) <= kTolerance;
    };
  if (!near(vertical_degrees, 40.0F)) {
    return AcousticsMode::UNKNOWN;
  }
  if (near(horizontal_degrees, 90.0F)) {
    return AcousticsMode::LOW_FREQUENCY;
  }
  if (near(horizontal_degrees, 40.0F)) {
    return AcousticsMode::HIGH_FREQUENCY;
  }
  return AcousticsMode::UNKNOWN;
}

// Nominal range image rates for diagnostics, before diagnostic_updater's 10%
// tolerance. Water Linked documents 5 Hz and 20 Hz, but low-frequency
// recordings image at 6.0 Hz, so each upper bound allows that 20% excess. The
// high-frequency rate has not yet been measured on hardware.
struct ImageRateBounds
{
  double minimum_hz;
  double maximum_hz;
};

[[nodiscard]] inline ImageRateBounds expected_image_rate(AcousticsMode mode)
{
  switch (mode) {
    case AcousticsMode::LOW_FREQUENCY:
      return {5.0, 6.0};
    case AcousticsMode::HIGH_FREQUENCY:
      return {20.0, 24.0};
    case AcousticsMode::UNKNOWN:
      break;
  }
  return {5.0, 24.0};
}

}  // namespace sonar3d
