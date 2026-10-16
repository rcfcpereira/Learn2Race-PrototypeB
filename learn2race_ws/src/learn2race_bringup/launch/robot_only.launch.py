"""Launch only the micro-ROS bridge on the robot.

This launch runs the micro-ROS agent and disables local joystick, teleop,
display, and all sensor nodes so the Pi only exposes the ROS /cmd_vel graph.
The RGB stick stays on (see start_leds below): it doubles as a connection
status display (micro_ros_link_led_node) for exactly this bridge-only case -
blinking red until the ESP32's micro-ROS session is established, solid green
once /odom starts flowing.
"""

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
    base_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("agent_baud", default_value="921600"),
            DeclareLaunchArgument("agent_verbose", default_value="6"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "serial_dev": LaunchConfiguration("serial_dev"),
                    "agent_baud": LaunchConfiguration("agent_baud"),
                    "agent_verbose": LaunchConfiguration("agent_verbose"),
                    "start_agent": "true",
                    "start_joy": "false",
                    "start_teleop": "false",
                    "start_leds": "true",
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
