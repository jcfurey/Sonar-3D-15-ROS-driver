// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>

namespace sonar3d
{

template<typename Publisher>
[[nodiscard]] bool has_subscribers(const std::shared_ptr<Publisher> & publisher)
{
  return publisher && (publisher->get_subscription_count() != 0 ||
         publisher->get_intra_process_subscription_count() != 0);
}

}  // namespace sonar3d
