"""Launch only the Raspberry Pi Camera v2 stereo pair and optional processing."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R17] camera_ros (libcamera node): camera_ros
#   [R18] Raspberry Pi camera software: libcamera
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The base launch owns camera nodes; this wrapper disables unrelated hardware.
    base_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_stereo_cameras", default_value="true"),
            DeclareLaunchArgument("start_stereo_processing", default_value="true"),
            DeclareLaunchArgument("left_camera", default_value="0"),
            DeclareLaunchArgument("right_camera", default_value="1"),
            DeclareLaunchArgument("stereo_width", default_value="640"),
            DeclareLaunchArgument("stereo_height", default_value="480"),
            DeclareLaunchArgument("stereo_sensor_mode", default_value="3280:2464"),
            DeclareLaunchArgument("stereo_base_frame", default_value="base_link"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                condition=IfCondition(LaunchConfiguration("start_stereo_cameras")),
                launch_arguments={
                    "start_agent": "false",
                    "start_joy": "false",
                    "start_teleop": "false",
                    "start_leds": "false",
                    "start_display": "false",
                    "start_imu": "false",
                    "start_tof": "false",
                    "start_lidar": "false",
                    "start_stereo_cameras": LaunchConfiguration("start_stereo_cameras"),
                    "start_stereo_processing": LaunchConfiguration("start_stereo_processing"),
                    "left_camera": LaunchConfiguration("left_camera"),
                    "right_camera": LaunchConfiguration("right_camera"),
                    "stereo_width": LaunchConfiguration("stereo_width"),
                    "stereo_height": LaunchConfiguration("stereo_height"),
                    "stereo_sensor_mode": LaunchConfiguration("stereo_sensor_mode"),
                    "stereo_base_frame": LaunchConfiguration("stereo_base_frame"),
                }.items(),
            ),
        ]
    )
