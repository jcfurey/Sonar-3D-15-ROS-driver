// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <span>

#include <cstdint>
#include <istream>
#include <ostream>

#include <nlohmann/json.hpp>

namespace sonar3d::timing
{
nlohmann::json describe_packet(std::span<const std::uint8_t> bytes);
// Offline transport: uint32 little-endian length followed by one exact RIP
// datagram; one flushed JSON line per request. No ROS context or networking.
void inspect_packets(std::istream & input, std::ostream & output);
}  // namespace sonar3d::timing
