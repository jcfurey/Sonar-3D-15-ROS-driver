# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Launch the native Water Linked Sonar 3D-15 ROS 2 driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare platform-facing settings and launch the C++ component executable."""
    defaults = {
        'sonar_ip': '192.168.194.96',
        'frame_id': 'sonar3d_link',
        'imu_frame_id': 'sonar3d_imu_link',
        'speed_of_sound': '0.0',
        'configure_sonar': 'true',
        'http_timeout': '5.0',
        'multicast_group': '224.0.0.96',
        'multicast_port': '4747',
        'multicast_interface': '0.0.0.0',
        'poll_period': '0.01',
        'max_packets_per_spin': '32',
        'use_sensor_timestamps': 'true',
        'publish_point_cloud': 'true',
        'publish_range_image': 'true',
        'publish_bitmap_images': 'true',
        'publish_imu': 'true',
        'diagnostics_period': '1.0',
        'packet_stale_timeout': '2.0',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    default_parameters = PathJoinSubstitution([
        FindPackageShare('sonar3d'),
        'config',
        'sonar3d.yaml',
    ])
    driver = Node(
        package='sonar3d',
        executable='sonar_publisher',
        name='sonar3d_driver',
        output='screen',
        parameters=[
            default_parameters,
            {
                name: LaunchConfiguration(name)
                for name in defaults
            },
        ],
    )
    return LaunchDescription(arguments + [driver])
