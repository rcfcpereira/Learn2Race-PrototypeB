"""Start the optional Twist PID controller."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R40] Astrom & Murray, Feedback Systems (PID): PID
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("learn2race_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "twist_pid.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_pid", default_value="true"),
            DeclareLaunchArgument("enabled", default_value="true"),
            DeclareLaunchArgument("setpoint_topic", default_value="/cmd_vel_setpoint"),
            DeclareLaunchArgument("measured_topic", default_value="/ddsm/measured_twist"),
            DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
            Node(
                package="learn2race_control",
                executable="twist_pid_controller_node",
                name="twist_pid_controller_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_pid")),
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "enabled": ParameterValue(
                            LaunchConfiguration("enabled"), value_type=bool
                        ),
                        "setpoint_topic": LaunchConfiguration("setpoint_topic"),
                        "measured_topic": LaunchConfiguration("measured_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    },
                ],
            ),
        ]
    )
