# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""View a standalone recording in its sonar frame, without vehicle TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start replay and an optional RViz viewer with matching topics and frame."""
    arguments = [
        DeclareLaunchArgument('file', description='Absolute path to the .sonar recording'),
        DeclareLaunchArgument('realtime_factor', default_value='1.0'),
        DeclareLaunchArgument('startup_delay', default_value='2.0'),
        DeclareLaunchArgument('frame_id', default_value='sonar3d_link'),
        DeclareLaunchArgument('imu_frame_id', default_value='sonar3d_imu_link'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    replay = Node(
        package='sonar3d',
        executable='sonar_replay',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        arguments=[
            '--file', LaunchConfiguration('file'),
            '--realtime-factor', LaunchConfiguration('realtime_factor'),
            '--startup-delay', LaunchConfiguration('startup_delay'),
            '--frame-id', LaunchConfiguration('frame_id'),
            '--imu-frame-id', LaunchConfiguration('imu_frame_id'),
            '--receive-time',
        ],
    )
    # The documented sensor-to-IMU offset gives RViz a real sensor TF tree,
    # including its fixed frame, without inventing a vehicle or odometry pose.
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        arguments=[
            '--x', '-0.022', '--y', '0.046', '--z', '0.003',
            '--frame-id', LaunchConfiguration('frame_id'),
            '--child-frame-id', LaunchConfiguration('imu_frame_id'),
        ],
    )
    viewer = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('namespace'),
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('sonar3d'), 'config', 'sonar_replay.rviz',
            ]),
            '-f', LaunchConfiguration('frame_id'),
        ],
    )
    close_viewer = RegisterEventHandler(OnProcessExit(
        target_action=viewer,
        on_exit=[EmitEvent(event=Shutdown(reason='RViz viewer closed'))],
    ))
    return LaunchDescription(arguments + [close_viewer, imu_tf, viewer, replay])
