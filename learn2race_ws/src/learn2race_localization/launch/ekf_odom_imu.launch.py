"""Start robot_localization EKF for wheel odometry + IMU yaw rate."""

# References (full entries: docs/references.md at the repository root):
#   [R45] Moore & Stouch, robot_localization EKF: robot_localization EKF
#   [R15] ROS 2 launch: launch
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("learn2race_localization")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "ekf_odom_imu.yaml"]
                ),
            ),
            DeclareLaunchArgument("start_ekf", default_value="true"),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_ekf")),
                parameters=[LaunchConfiguration("params_file")],
                remappings=[
                    ("odometry/filtered", "/odometry/filtered"),
                ],
            ),
        ]
    )
