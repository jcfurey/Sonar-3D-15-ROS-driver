// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "sonar3d/range_signal_pairing.hpp"

namespace
{

using Pairing = sonar3d::RangeSignalPairing<int>;

sonar3d::protocol::RangeImage range(std::uint32_t sequence)
{
  sonar3d::protocol::RangeImage image;
  image.header.sequence_id = sequence;
  image.width = 2;
  image.height = 1;
  image.pixels = {1, 2};
  return image;
}

sonar3d::protocol::BitmapImage signal(std::uint32_t sequence)
{
  sonar3d::protocol::BitmapImage image;
  image.header.sequence_id = sequence;
  image.width = 2;
  image.height = 1;
  image.pixels = {10, 20};
  return image;
}

TEST(RangeSignalPairing, PairsInEitherArrivalOrder)
{
  Pairing pairing;
  EXPECT_TRUE(pairing.add_range(range(1), 11).empty());
  ASSERT_NE(pairing.pending_context(), nullptr);
  EXPECT_EQ(*pairing.pending_context(), 11);
  auto ready = pairing.add_signal(signal(1));
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].context, 11);
  ASSERT_TRUE(ready[0].signal);
  EXPECT_EQ(ready[0].signal->header.sequence_id, 1U);
  EXPECT_EQ(pairing.pending_context(), nullptr);

  EXPECT_TRUE(pairing.add_signal(signal(2)).empty());
  ready = pairing.add_range(range(2), 22);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_TRUE(ready[0].signal);
  EXPECT_EQ(pairing.pending_context(), nullptr);
}

TEST(RangeSignalPairing, ReleasesRangeImagesWhoseSignalWasLost)
{
  Pairing pairing;
  EXPECT_TRUE(pairing.add_range(range(1), 1).empty());
  // The next range image releases the previous one unpaired.
  auto ready = pairing.add_range(range(2), 2);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].context, 1);
  EXPECT_FALSE(ready[0].signal);

  // A newer signal image shows the waiting one's partner was lost.
  ready = pairing.add_signal(signal(3));
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0].context, 2);
  EXPECT_FALSE(ready[0].signal);
  ready = pairing.add_range(range(3), 3);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_TRUE(ready[0].signal);

  // Explicit release after a timeout.
  EXPECT_TRUE(pairing.add_range(range(4), 4).empty());
  const auto released = pairing.release();
  ASSERT_TRUE(released);
  EXPECT_EQ(released->context, 4);
  EXPECT_FALSE(pairing.release());
}

TEST(RangeSignalPairing, IgnoresStaleAndMismatchedSignals)
{
  Pairing pairing;
  EXPECT_TRUE(pairing.add_range(range(5), 5).empty());
  EXPECT_TRUE(pairing.add_signal(signal(4)).empty());
  ASSERT_NE(pairing.pending_context(), nullptr);

  auto wrong_size = signal(5);
  wrong_size.width = 1;
  wrong_size.pixels = {10};
  EXPECT_TRUE(pairing.add_signal(wrong_size).empty());
  const auto ready = pairing.add_signal(signal(5));
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_TRUE(ready[0].signal);

  // A range image older than a waiting signal image is released at once.
  EXPECT_TRUE(pairing.add_signal(signal(9)).empty());
  const auto older = pairing.add_range(range(8), 8);
  ASSERT_EQ(older.size(), 1U);
  EXPECT_FALSE(older[0].signal);
  EXPECT_EQ(pairing.add_range(range(9), 9).size(), 1U);
}

TEST(RangeSignalPairing, HandlesSequenceWraparound)
{
  Pairing pairing;
  constexpr auto kLast = std::numeric_limits<std::uint32_t>::max();
  EXPECT_TRUE(pairing.add_range(range(kLast), 1).empty());
  // Sequence 0 follows the maximum, so the waiting image's signal was lost.
  const auto ready = pairing.add_signal(signal(0));
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_FALSE(ready[0].signal);
  EXPECT_EQ(pairing.add_range(range(0), 2).size(), 1U);
}

}  // namespace
