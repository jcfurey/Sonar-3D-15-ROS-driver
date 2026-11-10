// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT
#pragma once

#include <span>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sonar3d
{

struct RecordingState
{
  bool active{};
  std::string path;
  std::uint64_t packets{};
  std::uint64_t bytes{};
  std::uint64_t queued_bytes{};
  std::uint64_t dropped_packets{};
  std::string error;
};

// The receive thread only copies into a bounded queue. A separate worker
// writes exact datagrams, without parsing, filtering, or re-encoding them.
// An overflow or I/O failure ends capture and makes stop() report an error;
// an incomplete recording is never reported as successful.
class NativeRecorder
{
public:
  NativeRecorder() = default;
  ~NativeRecorder();
  NativeRecorder(const NativeRecorder &) = delete;
  NativeRecorder & operator=(const NativeRecorder &) = delete;

  // The parent directory must exist. Existing files are never overwritten.
  void start(const std::string & path, std::size_t queue_limit = 64U * 1024U * 1024U);
  bool enqueue(std::span<const std::uint8_t> packet);
  // Stops accepting packets, drains accepted packets, fsyncs, and joins.
  // Safe to repeat; the last capture's counters/error remain available.
  RecordingState stop();
  [[nodiscard]] RecordingState state() const;

private:
  void write_loop(int descriptor);
  void fail_locked(std::string error);

  std::mutex control_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<std::vector<std::uint8_t>> queue_;
  std::size_t queue_limit_{};
  RecordingState state_;
  std::thread worker_;
};

}  // namespace sonar3d
