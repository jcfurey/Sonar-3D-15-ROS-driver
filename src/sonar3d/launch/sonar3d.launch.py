"""Launch the Water Linked Sonar 3D-15 ROS driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare platform-facing settings and launch the live driver."""
    defaults = {
        'sonar_ip': '192.168.194.96',
        'frame_id': 'sonar3d_link',
        'speed_of_sound': '0.0',
        'configure_sonar': 'true',
        'http_timeout': '5.0',
        'multicast_group': '224.0.0.96',
        'multicast_port': '4747',
        'multicast_interface': '0.0.0.0',
        'poll_period': '0.01',
        'max_packets_per_spin': '32',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    driver = Node(
        package='sonar3d',
        executable='sonar_publisher',
        name='sonar3d_driver',
        output='screen',
        parameters=[{
            name: LaunchConfiguration(name)
            for name in defaults
        }],
    )
    return LaunchDescription(arguments + [driver])
