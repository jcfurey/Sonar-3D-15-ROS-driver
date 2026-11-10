// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT
#include "sonar3d/native_recorder.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace sonar3d
{

NativeRecorder::~NativeRecorder() {stop();}

void NativeRecorder::start(const std::string & path, std::size_t queue_limit)
{
  std::lock_guard control(control_mutex_);
  {
    std::lock_guard lock(mutex_);
    if (state_.active) {
      throw std::runtime_error("native recording is already active");
    }
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  const std::filesystem::path destination(path);
  if (!destination.is_absolute() || destination.extension() != ".sonar" || !queue_limit) {
    throw std::invalid_argument(
        "recording requires an absolute .sonar path and a positive queue limit");
  }
  const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  if (descriptor < 0) {
    throw std::system_error(errno, std::generic_category(),
        "cannot create native recording " + path);
  }
  {
    std::lock_guard lock(mutex_);
    queue_.clear();
    queue_limit_ = queue_limit;
    state_ = {};
    state_.active = true;
    state_.path = path;
  }
  try {
    worker_ = std::thread([this, descriptor] {write_loop(descriptor);});
  } catch (...) {
    ::close(descriptor);
    std::lock_guard lock(mutex_);
    fail_locked("could not start native recording worker");
    throw;
  }
}

bool NativeRecorder::enqueue(std::span<const std::uint8_t> packet)
{
  std::lock_guard lock(mutex_);
  if (!state_.active) {
    return false;
  }
  if (packet.empty()) {
    return true;
  }
  if (packet.size() > queue_limit_ - state_.queued_bytes) {
    ++state_.dropped_packets;
    fail_locked("native recording queue limit exceeded; capture is incomplete");
    ready_.notify_one();
    return false;
  }
  try {
    queue_.emplace_back(packet.begin(), packet.end());
  } catch (const std::exception & error) {
    ++state_.dropped_packets;
    fail_locked(std::string("native recording allocation failed: ") + error.what());
    ready_.notify_one();
    return false;
  }
  state_.queued_bytes += packet.size();
  ready_.notify_one();
  return true;
}

RecordingState NativeRecorder::stop()
{
  std::lock_guard control(control_mutex_);
  {
    std::lock_guard lock(mutex_);
    state_.active = false;
  }
  ready_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
  return state();
}

RecordingState NativeRecorder::state() const
{
  std::lock_guard lock(mutex_);
  return state_;
}

void NativeRecorder::fail_locked(std::string error)
{
  state_.active = false;
  if (state_.error.empty()) {
    state_.error = std::move(error);
  }
}

void NativeRecorder::write_loop(int descriptor)
{
  for (;; ) {
    std::vector<std::uint8_t> packet;
    {
      std::unique_lock lock(mutex_);
      ready_.wait(lock, [this] {return !state_.active || !queue_.empty();});
      if (queue_.empty()) {
        break;
      }
      packet = std::move(queue_.front());
      queue_.pop_front();
      state_.queued_bytes -= packet.size();
    }
    std::size_t written = 0;
    int error = 0;
    while (written < packet.size()) {
      const auto count = ::write(descriptor, packet.data() + written, packet.size() - written);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        error = count < 0 ? errno : EIO;
        break;
      }
      written += static_cast<std::size_t>(count);
    }
    {
      std::lock_guard lock(mutex_);
      state_.bytes += written;
      if (error) {
        state_.dropped_packets += 1 + queue_.size();
        queue_.clear();
        state_.queued_bytes = 0;
        fail_locked(std::string("native recording write failed: ") + std::strerror(error));
        break;
      }
      ++state_.packets;
    }
  }
  int result;
  do {
    result = ::fsync(descriptor);
  } while (result < 0 && errno == EINTR);
  if (result < 0) {
    const int error = errno;
    std::lock_guard lock(mutex_);
    fail_locked(std::string("native recording fsync failed: ") + std::strerror(error));
  }
  if (::close(descriptor) < 0) {
    const int error = errno;
    std::lock_guard lock(mutex_);
    fail_locked(std::string("native recording close failed: ") + std::strerror(error));
  }
}

}  // namespace sonar3d
