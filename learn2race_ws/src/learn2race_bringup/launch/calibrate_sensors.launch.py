"""Convenience launch for IMU and stereo camera calibration workflows."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
# More details: README.md of this package; learn2race_evaluation

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # This wrapper lets you choose IMU, stereo, or both calibration workflows,
    # while delegating the actual calibration logic to dedicated packages.
    imu_launch = PathJoinSubstitution(
        [
            FindPackageShare("learn2race_imu_calibration"),
            "launch",
            "imu_calibration.launch.py",
        ]
    )
    stereo_launch = PathJoinSubstitution(
        [
            FindPackageShare("learn2race_camera_calibration"),
            "launch",
            "stereo_camera_calibration.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("calibrate_imu", default_value="true"),
            DeclareLaunchArgument("calibrate_stereo", default_value="false"),
            DeclareLaunchArgument("imu_sample_count", default_value="500"),
            DeclareLaunchArgument("checkerboard_size", default_value="8x6"),
            DeclareLaunchArgument("checkerboard_square_m", default_value="0.025"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_launch),
                condition=IfCondition(LaunchConfiguration("calibrate_imu")),
                launch_arguments={
                    "sample_count": LaunchConfiguration("imu_sample_count"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stereo_launch),
                condition=IfCondition(LaunchConfiguration("calibrate_stereo")),
                launch_arguments={
                    "checkerboard_size": LaunchConfiguration("checkerboard_size"),
                    "checkerboard_square_m": LaunchConfiguration(
                        "checkerboard_square_m"
                    ),
                }.items(),
            ),
        ]
    )
