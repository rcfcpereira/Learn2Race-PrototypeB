"""Automatic room mapping launch for Learn2Race.

The launch reuses the normal robot bringup for hardware, then adds the TF,
SLAM Toolbox, and a slow LaserScan-based explorer needed to build a first room
map without Xbox input.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox: SLAM Toolbox
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("learn2race_bringup")

    # Reuse the main launch so wiring defaults stay centralized.
    base_launch = PathJoinSubstitution(
        [package_share, "launch", "xbox_drive.launch.py"]
    )
    slam_params_file = LaunchConfiguration("slam_params_file")
    explorer_params_file = LaunchConfiguration("explorer_params_file")

    serial_dev = LaunchConfiguration("serial_dev")
    agent_baud = LaunchConfiguration("agent_baud")
    agent_verbose = LaunchConfiguration("agent_verbose")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    scan_topic = LaunchConfiguration("scan_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    start_odom_tf = LaunchConfiguration("start_odom_tf")
    start_auto_explorer = LaunchConfiguration("start_auto_explorer")
    auto_explore_enabled = LaunchConfiguration("auto_explore_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_dev", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("agent_baud", default_value="921600"),
            DeclareLaunchArgument("agent_verbose", default_value="6"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("lidar_baud", default_value="230400"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("start_agent", default_value="true"),
            DeclareLaunchArgument("start_leds", default_value="true"),
            DeclareLaunchArgument("start_display", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_tof", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_odom_tf", default_value="true"),
            DeclareLaunchArgument("start_auto_explorer", default_value="true"),
            DeclareLaunchArgument("auto_explore_enabled", default_value="true"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "slam_toolbox_mapping.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "explorer_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "room_mapping_explorer.yaml"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "serial_dev": serial_dev,
                    "agent_baud": agent_baud,
                    "agent_verbose": agent_verbose,
                    "cmd_vel_topic": cmd_vel_topic,
                    "start_agent": LaunchConfiguration("start_agent"),
                    "start_joy": "false",
                    "start_teleop": "false",
                    "start_leds": LaunchConfiguration("start_leds"),
                    "start_display": LaunchConfiguration("start_display"),
                    "start_imu": LaunchConfiguration("start_imu"),
                    "start_tof": LaunchConfiguration("start_tof"),
                    "start_lidar": LaunchConfiguration("start_lidar"),
                    "lidar_port": lidar_port,
                    "lidar_baud": lidar_baud,
                    "lidar_scan_topic": scan_topic,
                    "lidar_base_frame": base_frame,
                    "start_stereo_cameras": "false",
                    "start_stereo_processing": "false",
                }.items(),
            ),
            # The ESP32 publishes /odom; this bridge provides odom -> base_link TF.
            Node(
                package="learn2race_odometry",
                executable="odom_tf_broadcaster_node",
                name="odom_tf_broadcaster_node",
                output="screen",
                condition=IfCondition(start_odom_tf),
                parameters=[
                    {
                        "odom_topic": odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "use_msg_frame_ids": False,
                        "use_msg_stamp": False,
                    }
                ],
            ),
            # SLAM Toolbox consumes /scan and odometry TF, then publishes map -> odom.
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params_file,
                    {
                        "scan_topic": scan_topic,
                        "odom_frame": odom_frame,
                        "map_frame": map_frame,
                        "base_frame": base_frame,
                    },
                ],
            ),
            # Slow wall-avoidance exploration for building an initial room map.
            Node(
                package="learn2race_bringup",
                executable="room_mapping_explorer_node",
                name="room_mapping_explorer_node",
                output="screen",
                condition=IfCondition(start_auto_explorer),
                parameters=[
                    explorer_params_file,
                    {
                        "scan_topic": scan_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "enabled": ParameterValue(
                            auto_explore_enabled, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
