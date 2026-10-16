"""Run Xbox teleop through the optional Twist PID controller."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R19] joy / teleop_twist_joy: gamepad teleoperation
#   [R40] Astrom & Murray, Feedback Systems (PID): PID
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )
    pid_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_control"), "launch", "twist_pid.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("setpoint_topic", default_value="/cmd_vel_setpoint"),
            DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={
                    "serial_dev": LaunchConfiguration("serial_dev"),
                    "joy_dev": LaunchConfiguration("joy_dev"),
                    # Teleop publishes the setpoint; PID publishes the firmware command.
                    "cmd_vel_topic": LaunchConfiguration("setpoint_topic"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pid_launch),
                launch_arguments={
                    "setpoint_topic": LaunchConfiguration("setpoint_topic"),
                    "output_topic": LaunchConfiguration("output_topic"),
                }.items(),
            ),
        ]
    )
