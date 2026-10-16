"""Launch the stereo cameras and start the ROS camera calibration GUI."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R34] Zhang (2000) camera calibration: calibration
#   [R17] camera_ros (libcamera node): camera_ros
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Start raw stereo cameras first, then launch the calibration GUI after a delay.
    stereo_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "stereo_cameras.launch.py"]
    )

    calibrator = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "camera_calibration",
            "cameracalibrator",
            "--size",
            LaunchConfiguration("checkerboard_size"),
            "--square",
            LaunchConfiguration("checkerboard_square_m"),
            "--approximate",
            LaunchConfiguration("approximate_sync"),
            "--ros-args",
            "--remap",
            "left:=/stereo/left/image_raw",
            "--remap",
            "right:=/stereo/right/image_raw",
            "--remap",
            "left_camera:=/stereo/left",
            "--remap",
            "right_camera:=/stereo/right",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_calibrator")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_camera", default_value="0"),
            DeclareLaunchArgument("right_camera", default_value="1"),
            DeclareLaunchArgument("stereo_width", default_value="640"),
            DeclareLaunchArgument("stereo_height", default_value="480"),
            DeclareLaunchArgument("checkerboard_size", default_value="8x6"),
            DeclareLaunchArgument("checkerboard_square_m", default_value="0.025"),
            DeclareLaunchArgument("approximate_sync", default_value="0.05"),
            DeclareLaunchArgument("start_calibrator", default_value="true"),
            DeclareLaunchArgument("calibrator_delay_s", default_value="4.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(stereo_launch),
                launch_arguments={
                    "start_stereo_cameras": "true",
                    "start_stereo_processing": "false",
                    "left_camera": LaunchConfiguration("left_camera"),
                    "right_camera": LaunchConfiguration("right_camera"),
                    "stereo_width": LaunchConfiguration("stereo_width"),
                    "stereo_height": LaunchConfiguration("stereo_height"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("calibrator_delay_s"),
                actions=[calibrator],
            ),
        ]
    )
