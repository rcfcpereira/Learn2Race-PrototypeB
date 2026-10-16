"""Launch every sensor without starting drive control or the Xbox stack."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Reuse the main launch so all sensor defaults stay in one place.
    base_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_display", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_tof", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_stereo_cameras", default_value="true"),
            DeclareLaunchArgument("start_stereo_processing", default_value="true"),
            DeclareLaunchArgument("display_i2c_bus", default_value="1"),
            DeclareLaunchArgument("display_i2c_address", default_value="60"),
            DeclareLaunchArgument("imu_i2c_bus", default_value="1"),
            DeclareLaunchArgument("imu_i2c_address", default_value="106"),
            DeclareLaunchArgument("tof_i2c_bus", default_value="1"),
            DeclareLaunchArgument("tof_mux_address", default_value="112"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("lidar_baud", default_value="230400"),
            DeclareLaunchArgument("left_camera", default_value="0"),
            DeclareLaunchArgument("right_camera", default_value="1"),
            DeclareLaunchArgument("stereo_width", default_value="640"),
            DeclareLaunchArgument("stereo_height", default_value="480"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "start_agent": "false",
                    "start_joy": "false",
                    "start_teleop": "false",
                    "start_leds": "false",
                    "start_display": LaunchConfiguration("start_display"),
                    "start_imu": LaunchConfiguration("start_imu"),
                    "start_tof": LaunchConfiguration("start_tof"),
                    "start_lidar": LaunchConfiguration("start_lidar"),
                    "start_stereo_cameras": LaunchConfiguration("start_stereo_cameras"),
                    "start_stereo_processing": LaunchConfiguration("start_stereo_processing"),
                    "display_i2c_bus": LaunchConfiguration("display_i2c_bus"),
                    "display_i2c_address": LaunchConfiguration("display_i2c_address"),
                    "imu_i2c_bus": LaunchConfiguration("imu_i2c_bus"),
                    "imu_i2c_address": LaunchConfiguration("imu_i2c_address"),
                    "tof_i2c_bus": LaunchConfiguration("tof_i2c_bus"),
                    "tof_mux_address": LaunchConfiguration("tof_mux_address"),
                    "lidar_port": LaunchConfiguration("lidar_port"),
                    "lidar_baud": LaunchConfiguration("lidar_baud"),
                    "left_camera": LaunchConfiguration("left_camera"),
                    "right_camera": LaunchConfiguration("right_camera"),
                    "stereo_width": LaunchConfiguration("stereo_width"),
                    "stereo_height": LaunchConfiguration("stereo_height"),
                }.items(),
            ),
        ]
    )
