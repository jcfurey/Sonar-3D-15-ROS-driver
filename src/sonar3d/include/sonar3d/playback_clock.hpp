// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace sonar3d
{

// Anchor recorded time to the steady clock so decoding and publishing consume
// the interval budget. Older samples within an interleaved IMU batch must not
// move that anchor. A backward jump exceeding one second starts a new segment.
class PlaybackClock
{
public:
  using Clock = std::chrono::steady_clock;

  explicit PlaybackClock(double realtime_factor)
  : realtime_factor_(realtime_factor)
  {
    if (!std::isfinite(realtime_factor) || realtime_factor <= 0.0) {
      throw std::invalid_argument("realtime_factor must be finite and greater than zero");
    }
  }

  [[nodiscard]] Clock::time_point deadline(
    std::optional<std::int64_t> sensor_nanoseconds, Clock::time_point now)
  {
    if (!sensor_nanoseconds) {
      return now;
    }
    const auto stamp = *sensor_nanoseconds;
    constexpr std::int64_t kResetThreshold = 1'000'000'000;
    if (!sensor_origin_ || static_cast<long double>(latest_stamp_) - stamp > kResetThreshold) {
      sensor_origin_ = stamp;
      latest_stamp_ = stamp;
      wall_origin_ = now;
      return now;
    }
    latest_stamp_ = std::max(latest_stamp_, stamp);
    const auto elapsed = std::chrono::duration<long double, std::nano>(
      (static_cast<long double>(stamp) - *sensor_origin_) / realtime_factor_);
    if (elapsed <= Clock::duration::zero()) {
      return now;
    }
    if (elapsed >= Clock::time_point::max() - wall_origin_) {
      return Clock::time_point::max();
    }
    return std::max(now, wall_origin_ + std::chrono::duration_cast<Clock::duration>(elapsed));
  }

private:
  double realtime_factor_;
  std::optional<std::int64_t> sensor_origin_;
  std::int64_t latest_stamp_{};
  Clock::time_point wall_origin_{};
};

}  // namespace sonar3d
