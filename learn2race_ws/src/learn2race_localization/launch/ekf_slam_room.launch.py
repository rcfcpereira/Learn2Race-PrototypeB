"""Run automatic room SLAM with EKF-owned odom -> base_link TF."""

# References (full entries: docs/references.md at the repository root):
#   [R45] Moore & Stouch, robot_localization EKF: robot_localization EKF
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox: SLAM Toolbox
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "slam_room.launch.py"]
    )
    ekf_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_localization"), "launch", "ekf_odom_imu.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("start_auto_explorer", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "serial_dev": LaunchConfiguration("serial_dev"),
                    "lidar_port": LaunchConfiguration("lidar_port"),
                    "start_auto_explorer": LaunchConfiguration("start_auto_explorer"),
                    # EKF publishes odom -> base_link, so disable the raw odom TF bridge.
                    "start_odom_tf": "false",
                    "start_imu": "true",
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(ekf_launch)),
        ]
    )
