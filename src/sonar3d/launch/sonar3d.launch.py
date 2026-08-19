"""Launch the Water Linked Sonar 3D-15 ROS driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare platform-facing settings and launch the live driver."""
    arguments = [
        DeclareLaunchArgument('sonar_ip', default_value='192.168.194.96'),
        DeclareLaunchArgument('frame_id', default_value='sonar3d_link'),
        DeclareLaunchArgument('speed_of_sound', default_value='0'),
        DeclareLaunchArgument('configure_sonar', default_value='true'),
        DeclareLaunchArgument('http_timeout', default_value='5.0'),
        DeclareLaunchArgument('multicast_interface', default_value='0.0.0.0'),
    ]
    driver = Node(
        package='sonar3d',
        executable='sonar_publisher',
        name='sonar3d_driver',
        output='screen',
        parameters=[{
            'sonar_ip': LaunchConfiguration('sonar_ip'),
            'frame_id': LaunchConfiguration('frame_id'),
            'speed_of_sound': LaunchConfiguration('speed_of_sound'),
            'configure_sonar': LaunchConfiguration('configure_sonar'),
            'http_timeout': LaunchConfiguration('http_timeout'),
            'multicast_interface': LaunchConfiguration('multicast_interface'),
        }],
    )
    return LaunchDescription(arguments + [driver])
