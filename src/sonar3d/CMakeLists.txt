cmake_minimum_required(VERSION 3.16)
project(sonar3d)

# Resolve the ROS and ament dependencies declared in package.xml.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install the Python module through ament_cmake_python. This removes the
# package-owned setup.py/install_data path that broke merged symlink installs.
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/sonar_publisher
    scripts/sonar_to_bag
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # This repository combines upstream and downstream work without a uniform
  # source-header copyright convention. Keep the prior copyright-lint skip.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_conversions
    test/test_conversions.py
  )
  ament_add_pytest_test(
    test_http_api
    test/test_http_api.py
  )
  ament_add_pytest_test(
    test_recording
    test/test_recording.py
  )
  ament_add_pytest_test(
    test_ros_messages
    test/test_ros_messages.py
  )
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
