"""Launch only the LSM6DS3 IMU plus the stationary calibration helper."""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R12] REP 103, units and axes: IMU axes
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Reuse the IMU node from the main launch and add the calibration collector.
    base_launch = PathJoinSubstitution(
        [FindPackageShare("learn2race_bringup"), "launch", "xbox_drive.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_i2c_bus", default_value="1"),
            DeclareLaunchArgument("imu_i2c_address", default_value="106"),
            DeclareLaunchArgument("sample_count", default_value="500"),
            DeclareLaunchArgument(
                "output_path",
                default_value="~/.ros/learn2race/imu_calibration.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "start_agent": "false",
                    "start_joy": "false",
                    "start_teleop": "false",
                    "start_leds": "false",
                    "start_display": "false",
                    "start_imu": "true",
                    "start_tof": "false",
                    "start_lidar": "false",
                    "start_stereo_cameras": "false",
                    "start_stereo_processing": "false",
                    "imu_i2c_bus": LaunchConfiguration("imu_i2c_bus"),
                    "imu_i2c_address": LaunchConfiguration("imu_i2c_address"),
                }.items(),
            ),
            Node(
                package="learn2race_bringup",
                executable="imu_calibration_node",
                name="imu_calibration_node",
                output="screen",
                parameters=[
                    {
                        "sample_count": ParameterValue(
                            LaunchConfiguration("sample_count"), value_type=int
                        ),
                        "output_path": LaunchConfiguration("output_path"),
                    }
                ],
            ),
        ]
    )
