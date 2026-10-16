"""Start the Learn2Race odometry TF broadcaster."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R13] REP 105, coordinate frames: frames
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("learn2race_odometry")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "odometry.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_odom_tf", default_value="true"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("use_msg_frame_ids", default_value="false"),
            DeclareLaunchArgument("use_msg_stamp", default_value="false"),
            Node(
                package="learn2race_odometry",
                executable="odom_tf_broadcaster_node",
                name="odom_tf_broadcaster_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_odom_tf")),
                parameters=[
                    params_file,
                    {
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "base_frame": LaunchConfiguration("base_frame"),
                        "use_msg_frame_ids": ParameterValue(
                            LaunchConfiguration("use_msg_frame_ids"),
                            value_type=bool,
                        ),
                        "use_msg_stamp": ParameterValue(
                            LaunchConfiguration("use_msg_stamp"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
