"""Launch only the devices on Raspberry Pi I2C bus 1."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
# More details: README.md of this package; learn2race_hardware

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # I2C-only mode is useful for bench testing wiring before running motors.
    base_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_display", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_tof", default_value="true"),
            DeclareLaunchArgument("display_i2c_bus", default_value="1"),
            DeclareLaunchArgument("display_i2c_address", default_value="60"),
            DeclareLaunchArgument("imu_i2c_bus", default_value="1"),
            DeclareLaunchArgument("imu_i2c_address", default_value="106"),
            DeclareLaunchArgument("tof_i2c_bus", default_value="1"),
            DeclareLaunchArgument("tof_mux_address", default_value="112"),
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
                    "start_lidar": "false",
                    "start_stereo_cameras": "false",
                    "start_stereo_processing": "false",
                    "display_i2c_bus": LaunchConfiguration("display_i2c_bus"),
                    "display_i2c_address": LaunchConfiguration("display_i2c_address"),
                    "imu_i2c_bus": LaunchConfiguration("imu_i2c_bus"),
                    "imu_i2c_address": LaunchConfiguration("imu_i2c_address"),
                    "tof_i2c_bus": LaunchConfiguration("tof_i2c_bus"),
                    "tof_mux_address": LaunchConfiguration("tof_mux_address"),
                }.items(),
            ),
        ]
    )
