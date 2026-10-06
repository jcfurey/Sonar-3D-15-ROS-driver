// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "sonar3d/protocol.hpp"

namespace sonar3d
{

// Pairs each range image with the signal-strength image of the same shot so
// the point cloud can carry intensity. The sonar sends both back to back with
// the same sequence ID and size. A range image whose signal image is lost is
// released unpaired when the next range image arrives, when a newer signal
// image arrives, or when the caller calls release() after a short wait.
template<typename Context>
class RangeSignalPairing
{
public:
  struct Pair
  {
    protocol::RangeImage range;
    Context context;
    // Empty when no signal image arrived for this shot.
    std::optional<protocol::BitmapImage> signal;
  };

  // Returns the pairs that are complete, oldest first.
  [[nodiscard]] std::vector<Pair> add_range(protocol::RangeImage range, Context context)
  {
    std::vector<Pair> ready;
    if (pending_range_) {
      ready.push_back(take_pending_range());
    }
    if (pending_signal_ && matches(range, *pending_signal_)) {
      ready.push_back(Pair{std::move(range), std::move(context), std::move(pending_signal_)});
      pending_signal_.reset();
      return ready;
    }
    if (pending_signal_ && newer(*pending_signal_, range)) {
      // The signal image belongs to a later shot; this one's was lost.
      ready.push_back(Pair{std::move(range), std::move(context), std::nullopt});
      return ready;
    }
    pending_signal_.reset();
    pending_range_.emplace(Pair{std::move(range), std::move(context), std::nullopt});
    return ready;
  }

  [[nodiscard]] std::vector<Pair> add_signal(protocol::BitmapImage signal)
  {
    std::vector<Pair> ready;
    if (pending_range_ && matches(pending_range_->range, signal)) {
      pending_range_->signal = std::move(signal);
      ready.push_back(take_pending_range());
    } else if (pending_range_ && newer(signal, pending_range_->range)) {
      ready.push_back(take_pending_range());
      pending_signal_ = std::move(signal);
    } else if (!pending_range_) {
      pending_signal_ = std::move(signal);
    }
    // Otherwise the signal image is older than the waiting range image: stale.
    return ready;
  }

  // Release a waiting range image without its signal image.
  [[nodiscard]] std::optional<Pair> release()
  {
    if (!pending_range_) {
      return std::nullopt;
    }
    return take_pending_range();
  }

  [[nodiscard]] const Context * pending_context() const
  {
    return pending_range_ ? &pending_range_->context : nullptr;
  }

  void clear()
  {
    pending_range_.reset();
    pending_signal_.reset();
  }

private:
  [[nodiscard]] static bool matches(
    const protocol::RangeImage & range, const protocol::BitmapImage & signal)
  {
    return range.header.sequence_id == signal.header.sequence_id &&
           range.width == signal.width && range.height == signal.height &&
           signal.pixels.size() == range.pixels.size();
  }

  // Sequence IDs wrap around, so compare by signed distance.
  template<typename First, typename Second>
  [[nodiscard]] static bool newer(const First & first, const Second & second)
  {
    return static_cast<std::int32_t>(first.header.sequence_id - second.header.sequence_id) > 0;
  }

  Pair take_pending_range()
  {
    std::optional<Pair> taken;
    taken.swap(pending_range_);
    return std::move(*taken);
  }

  std::optional<Pair> pending_range_;
  std::optional<protocol::BitmapImage> pending_signal_;
};

}  // namespace sonar3d
