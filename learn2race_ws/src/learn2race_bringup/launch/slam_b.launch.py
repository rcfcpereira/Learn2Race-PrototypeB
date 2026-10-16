"""2D LiDAR SLAM for Prototype B (slam_toolbox, online asynchronous mapping).

Run alongside the robot bringup, which provides /scan, /odom and the
odom -> base_link -> base_laser transforms:

    ros2 launch learn2race_bringup prototype_b.launch.py teleop:=false
    ros2 launch learn2race_bringup slam_b.launch.py

Drive slowly with the gamepad (from the PC: teleop_pc.launch.py). Save the map:

    ros2 run nav2_map_server map_saver_cli -f ~/maps/room

The map is only as good as base_link -> base_laser in sensor_frames_b.yaml and
the wheel odometry; turn slowly, since scan matching tolerates translation
errors far better than fast rotation.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox: SLAM Toolbox
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('learn2race_bringup'), 'config', 'slam_toolbox_mapping.yaml'])
    slam = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file', default_value=params),
        # Constant beam count for slam_toolbox (see scan_fixed_node.py); the config reads /scan_fixed.
        Node(package='learn2race_bringup', executable='scan_fixed_node', name='scan_fixed',
             parameters=[{'beams': 453}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),
    ])
