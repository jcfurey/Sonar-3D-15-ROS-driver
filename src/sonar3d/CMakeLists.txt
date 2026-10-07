cmake_minimum_required(VERSION 3.16)
project(sonar3d VERSION 0.6.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(CURL REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nlohmann_json 3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ZLIB REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/ImageMetadata.msg" DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(sonar3d_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

# Ubuntu's libsnappy-dev exports Snappy::snappy on newer releases. Retain a
# small fallback for Jazzy-era distributions that only install headers/library.
find_package(Snappy QUIET CONFIG)
if(NOT TARGET Snappy::snappy)
  find_path(SNAPPY_INCLUDE_DIR NAMES snappy.h REQUIRED)
  find_library(SNAPPY_LIBRARY NAMES snappy REQUIRED)
  add_library(Snappy::snappy UNKNOWN IMPORTED)
  set_target_properties(
    Snappy::snappy
    PROPERTIES
      IMPORTED_LOCATION "${SNAPPY_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${SNAPPY_INCLUDE_DIR}"
  )
endif()

protobuf_generate_cpp(
  SONAR3D_PROTO_SOURCES
  SONAR3D_PROTO_HEADERS
  proto/WaterLinkedSonarIntegrationProtocol.proto
)

add_library(
  sonar3d_protocol SHARED
  src/protocol.cpp
  ${SONAR3D_PROTO_SOURCES}
  ${SONAR3D_PROTO_HEADERS}
)
target_compile_features(sonar3d_protocol PUBLIC cxx_std_20)
target_include_directories(
  sonar3d_protocol
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}"
)
target_link_libraries(
  sonar3d_protocol
  PRIVATE protobuf::libprotobuf Snappy::snappy ZLIB::ZLIB
)

file(GLOB_RECURSE sonar3d_sources CONFIGURE_DEPENDS
  "src/*.cpp" "include/*.hpp" "proto/*.proto" "msg/*.msg")
list(APPEND sonar3d_sources "${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt"
  "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
list(SORT sonar3d_sources)
set(sonar3d_source_hashes "")
foreach(source IN LISTS sonar3d_sources)
  file(SHA256 "${source}" source_hash)
  file(RELATIVE_PATH source_name "${CMAKE_CURRENT_SOURCE_DIR}" "${source}")
  string(APPEND sonar3d_source_hashes "${source_name}:${source_hash}\n")
endforeach()
string(SHA256 sonar3d_source_sha256 "${sonar3d_source_hashes}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${sonar3d_sources})
add_library(sonar3d_ros SHARED src/conversions.cpp)
target_compile_definitions(sonar3d_ros PRIVATE SONAR3D_VERSION="${PROJECT_VERSION}"
  SONAR3D_SOURCE_SHA256="${sonar3d_source_sha256}")
target_compile_features(sonar3d_ros PUBLIC cxx_std_20)
target_include_directories(
  sonar3d_ros
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  sonar3d_ros
  PUBLIC
    sonar3d_protocol
    ${builtin_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${sonar3d_typesupport}
)

add_library(sonar3d_io SHARED src/http_client.cpp src/udp_receiver.cpp)
target_compile_features(sonar3d_io PUBLIC cxx_std_20)
target_include_directories(
  sonar3d_io
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  sonar3d_io PUBLIC sonar3d_protocol PRIVATE CURL::libcurl nlohmann_json::nlohmann_json
)

generate_parameter_library(
  sonar3d_parameters
  src/sonar3d_parameters.yaml
)

add_library(sonar3d_component SHARED src/sonar_driver.cpp)
target_compile_features(sonar3d_component PUBLIC cxx_std_20)
target_include_directories(
  sonar3d_component
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  sonar3d_component
  PUBLIC
    sonar3d_io
    sonar3d_parameters
    sonar3d_protocol
    sonar3d_ros
    ${builtin_interfaces_TARGETS}
    ${diagnostic_msgs_TARGETS}
    diagnostic_updater::diagnostic_updater
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${sensor_msgs_TARGETS}
    tf2_ros::tf2_ros
  PRIVATE
    ${geometry_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
    rclcpp_components::component
)
rclcpp_components_register_nodes(sonar3d_component "sonar3d::SonarDriver")

add_executable(sonar_publisher src/sonar_publisher.cpp)
target_link_libraries(sonar_publisher PRIVATE sonar3d_component)

foreach(player_target IN ITEMS sonar_replay sonar_to_bag)
  add_executable(${player_target} src/recording_player.cpp)
  target_compile_features(${player_target} PRIVATE cxx_std_20)
  target_link_libraries(
    ${player_target}
    PRIVATE
      sonar3d_protocol
      sonar3d_ros
      rclcpp::rclcpp
      rosbag2_cpp::rosbag2_cpp
      rosbag2_storage::rosbag2_storage
      ${sensor_msgs_TARGETS}
  )
endforeach()

install(
  TARGETS
    sonar3d_protocol
    sonar3d_ros
    sonar3d_io
    sonar3d_component
    sonar3d_parameters
  EXPORT sonar3dTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS sonar_publisher sonar_replay sonar_to_bag RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY proto/ DESTINATION share/${PROJECT_NAME}/proto)
install(PROGRAMS tools/sonar_image_viewer.py DESTINATION lib/${PROJECT_NAME}
  RENAME sonar_image_viewer)
install(PROGRAMS tools/sonar_image_publisher.py DESTINATION lib/${PROJECT_NAME}
  RENAME sonar_image_publisher)
install(FILES tools/sonar_image_science.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # The ament wrapper intentionally omits external include paths, so cppcheck
  # cannot expand GoogleTest's TEST macro. Analyze production code here; the
  # test translation units are compiled with the same warning policy above.
  set(
    ament_cmake_cppcheck_ADDITIONAL_EXCLUDE
    test/test_conversions.cpp
    test/test_http_client.cpp
    test/test_protocol.cpp
    test/test_playback_clock.cpp
    test/test_range_signal_pairing.cpp
    test/test_sensor_clock.cpp
    test/test_udp_receiver.cpp
    test/test_driver_stream.cpp
    test/test_recording_player.cpp
  )
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_image_science test/test_image_science.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol sonar3d_protocol)

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions sonar3d_ros)

  ament_add_gtest(test_http_client test/test_http_client.cpp)
  target_link_libraries(test_http_client sonar3d_io)

  ament_add_gtest(test_playback_clock test/test_playback_clock.cpp)
  target_link_libraries(test_playback_clock sonar3d_protocol)

  ament_add_gtest(test_range_signal_pairing test/test_range_signal_pairing.cpp)
  target_link_libraries(test_range_signal_pairing sonar3d_protocol)

  ament_add_gtest(test_sensor_clock test/test_sensor_clock.cpp)
  target_include_directories(
    test_sensor_clock PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")

  ament_add_gtest(test_udp_receiver test/test_udp_receiver.cpp)
  target_link_libraries(test_udp_receiver sonar3d_io)

  ament_add_gtest(test_driver_stream test/test_driver_stream.cpp TIMEOUT 60)
  target_link_libraries(test_driver_stream sonar3d_component)

  ament_add_gtest(test_recording_player test/test_recording_player.cpp TIMEOUT 30)
  target_link_libraries(
    test_recording_player
    sonar3d_ros rclcpp::rclcpp rosbag2_cpp::rosbag2_cpp protobuf::libprotobuf ZLIB::ZLIB
  )
  target_include_directories(test_recording_player PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
  target_compile_definitions(
    test_recording_player PRIVATE TEST_SONAR_REPLAY_PATH="$<TARGET_FILE:sonar_replay>"
  )
  add_dependencies(test_recording_player sonar_replay)

  # Opt-in measurement tool: timings are reported, never asserted by CTest.
  add_executable(benchmark_stream EXCLUDE_FROM_ALL test/benchmark_stream.cpp)
  target_link_libraries(benchmark_stream sonar3d_component)
endif()

ament_export_targets(sonar3dTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  diagnostic_updater
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2_ros
  rosidl_default_runtime
)
ament_package()
