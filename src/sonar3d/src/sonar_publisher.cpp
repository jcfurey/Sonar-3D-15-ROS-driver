// Copyright 2026 Sonar 3D-15 ROS Driver contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
// SPDX-License-Identifier: MIT

// Standalone executable for the SonarDriver component. Unlike the generic
// component main, it reports invalid parameters as an error and exits with a
// failure status instead of aborting.

#include <cstdlib>
#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sonar3d/sonar_driver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = EXIT_SUCCESS;
  try {
    const auto driver = std::make_shared<sonar3d::SonarDriver>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(driver->get_node_base_interface());
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sonar3d_driver"), "%s", error.what());
    status = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return status;
}
