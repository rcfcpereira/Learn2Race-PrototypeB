"""Launch only micro-ROS serial control and Xbox teleoperation."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Include the main launch with every optional sensor disabled.
    base_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("agent_baud", default_value="921600"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "serial_dev": LaunchConfiguration("serial_dev"),
                    "agent_baud": LaunchConfiguration("agent_baud"),
                    "joy_dev": LaunchConfiguration("joy_dev"),
                    "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                    "start_agent": "true",
                    "start_joy": "true",
                    "start_teleop": "true",
                    "start_leds": "false",
                    "start_display": "false",
                    "start_imu": "false",
                    "start_tof": "false",
                    "start_lidar": "false",
                    "start_stereo_cameras": "false",
                    "start_stereo_processing": "false",
                }.items(),
            ),
        ]
    )
