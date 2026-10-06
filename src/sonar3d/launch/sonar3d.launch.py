# Copyright 2026 Sonar 3D-15 ROS Driver contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
# SPDX-License-Identifier: MIT

"""Launch the native Water Linked Sonar 3D-15 ROS 2 driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, LoadComposableNodes
from launch_ros.descriptions import ComposableLifecycleNode
from launch_ros.substitutions import FindPackageShare

# Driver parameters that may be overridden from the command line, with the
# type each one is declared with. An empty argument keeps the params_file
# value, so the YAML file stays authoritative unless an override is given.
PARAMETER_TYPES = {
    'sonar_ip': str,
    'fallback_ip': str,
    'frame_id': str,
    'imu_frame_id': str,
    'speed_of_sound': float,
    'configure_sonar': bool,
    'http_timeout': float,
    'multicast_group': str,
    'multicast_port': int,
    'multicast_interface': str,
    'udp_receive_buffer_size': int,
    'use_sensor_timestamps': bool,
    'max_sensor_clock_offset': float,
    'publish_tf': bool,
    'publish_point_cloud': bool,
    'publish_range_image': bool,
    'publish_bitmap_images': bool,
    'publish_imu': bool,
    'linear_acceleration_stddev': float,
    'angular_velocity_stddev': float,
    'packet_stale_timeout': float,
}


def _convert(name, text, declared_type):
    """Convert an argument to its declared type so rclcpp accepts it."""
    if declared_type is bool:
        if text.lower() not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false, got {text!r}')
        return text.lower() == 'true'
    try:
        return declared_type(text)
    except ValueError as error:
        raise ValueError(f'{name} must be a {declared_type.__name__}, got {text!r}') from error


def _driver(context):
    # The driver's autostart parameter configures and activates it once it
    # spins. launch_ros's own composable-node autostart mis-resolves
    # namespaced node names in Jazzy, so it is not used.
    autostart = LaunchConfiguration('autostart').perform(context)
    overrides = {'autostart': _convert('autostart', autostart, bool)}
    for name, declared_type in PARAMETER_TYPES.items():
        text = LaunchConfiguration(name).perform(context)
        if text:
            overrides[name] = _convert(name, text, declared_type)
    parameters = [LaunchConfiguration('params_file').perform(context), overrides]
    namespace = LaunchConfiguration('namespace').perform(context)
    container = LaunchConfiguration('container').perform(context)

    if container:
        return [LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=[ComposableLifecycleNode(
                package='sonar3d',
                plugin='sonar3d::SonarDriver',
                name='sonar3d_driver',
                namespace=namespace,
                parameters=parameters,
                extra_arguments=[{'use_intra_process_comms': True}],
            )],
        )]
    return [LifecycleNode(
        package='sonar3d',
        executable='sonar_publisher',
        name='sonar3d_driver',
        namespace=namespace,
        output='screen',
        parameters=parameters,
        ros_arguments=['--log-level', LaunchConfiguration('log_level').perform(context)],
    )]


def generate_launch_description():
    """Declare the parameter file, optional overrides, and how to run the driver."""
    arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('sonar3d'), 'config', 'sonar3d.yaml']),
            description='Driver parameter file'),
        DeclareLaunchArgument(
            'namespace', default_value='sonar3d',
            description='Namespace for the driver and its topics'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the managed driver on start; false leaves it '
                        'unconfigured for a lifecycle manager'),
        DeclareLaunchArgument(
            'container', default_value='',
            description='Load the driver into this component container instead of its own '
                        'process, with intra-process communication enabled'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Driver log level when it runs in its own process'),
    ]
    arguments += [
        DeclareLaunchArgument(
            name, default_value='',
            description=f'Override {name} from params_file ({declared_type.__name__})')
        for name, declared_type in PARAMETER_TYPES.items()
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_driver)])
