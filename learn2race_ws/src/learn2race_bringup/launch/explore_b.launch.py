"""Autonomous room exploration for Prototype B (room_mapping_explorer_node).

Run with the robot and SLAM up (robot.sh does all three: ~/robot.sh explore [SECONDS]):

    ros2 launch learn2race_bringup prototype_b.launch.py teleop:=false ekf:=true
    ros2 launch learn2race_bringup slam_b.launch.py
    ros2 launch learn2race_bringup explore_b.launch.py max_run_s:=180

The explorer drives through cmd_vel_mux (autonomy/cmd_vel, arming the mux only while it runs);
the gamepad overrides it, and it stops after max_run_s. Parameters: room_mapping_explorer_b.yaml.
"""
# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox: SLAM Toolbox
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('learn2race_bringup'), 'config', 'room_mapping_explorer_b.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('max_run_s', default_value='180.0'),
        DeclareLaunchArgument('mode', default_value='wall_follow'),
        Node(package='learn2race_bringup', executable='room_mapping_explorer_node',
             name='room_mapping_explorer_node', output='screen',
             parameters=[params, {
                 'max_run_s': ParameterValue(LaunchConfiguration('max_run_s'), value_type=float),
                 'mode': LaunchConfiguration('mode'),
             }]),
    ])
