// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <stdexcept>

namespace sonar3d
{

// Maps an unsynchronized sonar clock onto ROS time with one offset shared by
// every product. Transport and processing delay only make a message arrive
// later, so the smallest receive-minus-sensor difference is the best estimate
// of the clock offset (plus the minimum latency). Taking the minimum over two
// rolling windows rejects latency spikes while following slow drift between
// the clocks, and keeps sensor spacing between images and IMU samples exact.
class SensorClockOffset
{
public:
  SensorClockOffset(std::int64_t window_nanoseconds, std::int64_t reset_threshold_nanoseconds)
  : window_(window_nanoseconds), reset_threshold_(reset_threshold_nanoseconds)
  {
    if (window_ <= 0 || reset_threshold_ <= 0) {
      throw std::invalid_argument("clock offset window and reset threshold must be positive");
    }
  }

  // A positive jump may be delivery delay rather than a backward clock step.
  // Require consistent observations spanning the reset threshold before
  // restarting; isolated or burst-delivered late packets cannot reset the fit.
  [[nodiscard]] std::int64_t update(std::int64_t receive_minus_sensor, std::int64_t receive_time)
  {
    if (current_ && receive_minus_sensor - estimate() > reset_threshold_) {
      const auto tolerance = reset_threshold_ / 10;
      if (!step_ || std::abs(receive_minus_sensor - *step_) > tolerance) {
        step_ = receive_minus_sensor;
        step_start_ = receive_time;
        step_samples_ = 1;
      } else {
        ++step_samples_;
      }
      if (step_samples_ < 3 || receive_time - step_start_ < reset_threshold_) {
        return estimate();
      }
      reset();
    } else {
      step_.reset();
    }
    if (!current_) {
      current_ = receive_minus_sensor;
      previous_.reset();
      window_start_ = receive_time;
    } else if (receive_time - window_start_ >= window_) {
      previous_ = current_;
      current_ = receive_minus_sensor;
      window_start_ = receive_time;
    } else {
      current_ = std::min(*current_, receive_minus_sensor);
    }
    return estimate();
  }

  void reset()
  {
    current_.reset();
    previous_.reset();
    step_.reset();
  }

private:
  [[nodiscard]] std::int64_t estimate() const
  {
    return previous_ ? std::min(*previous_, *current_) : *current_;
  }

  std::int64_t window_;
  std::int64_t reset_threshold_;
  std::optional<std::int64_t> current_;
  std::optional<std::int64_t> previous_;
  std::optional<std::int64_t> step_;
  std::int64_t step_start_{};
  unsigned step_samples_{};
  std::int64_t window_start_{};
};

}  // namespace sonar3d
