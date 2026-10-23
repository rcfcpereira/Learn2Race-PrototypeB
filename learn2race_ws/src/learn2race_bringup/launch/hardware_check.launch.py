"""First-line hardware check (NFR5): bringup outside competition, then the check.

    ros2 launch learn2race_bringup hardware_check.launch.py prototype:=b
    ros2 launch learn2race_bringup hardware_check.launch.py prototype:=b wheels_lifted:=true

Usually started with `~/robot.sh check`. Starts the robot's normal bringup with
competition off (LiDAR and display on, teleoperation off), waits for it to come
up, runs learn2race_evaluation hardware_check, and stops everything when the
report is written. The motor test runs only with wheels_lifted:=true: put the
robot on a stand first.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R3]  Learn2Race dissertation, requirements FR/NFR: NFR5
# More details: README.md of this package

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Bringup arguments per prototype: competition off, so nothing else drives the
# LED or the wheels, and every sensor the check expects is started.
BRINGUP_ARGUMENTS = {
    'a': {'competition': 'false', 'sensor_host': 'pi', 'lidar': 'true', 'display': 'true',
          'teleop': 'false', 'allow_teleop': 'false'},
    'b': {'competition': 'false', 'teleop': 'false'},
}


def launch_setup(context):
    prototype = LaunchConfiguration('prototype').perform(context).lower()
    if prototype not in BRINGUP_ARGUMENTS:
        raise RuntimeError(f"prototype must be a or b, not '{prototype}'")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('learn2race_bringup'), 'launch',
            f'prototype_{prototype}.launch.py')),
        launch_arguments=BRINGUP_ARGUMENTS[prototype].items())
    check = Node(
        package='learn2race_evaluation', executable='hardware_check',
        name='hardware_check', output='screen', emulate_tty=True,
        parameters=[{
            'wheels_lifted': LaunchConfiguration('wheels_lifted').perform(context) == 'true',
            'robot_yaml': LaunchConfiguration('robot_yaml').perform(context),
        }])
    return [
        bringup,
        # Let the agent connect and the safety monitor finish its start-up grace.
        TimerAction(period=float(LaunchConfiguration('start_delay').perform(context)),
                    actions=[check]),
        RegisterEventHandler(OnProcessExit(
            target_action=check,
            on_exit=[EmitEvent(event=Shutdown(reason='hardware check finished'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('prototype', default_value='b', choices=['a', 'b', 'A', 'B']),
        DeclareLaunchArgument('wheels_lifted', default_value='false',
                              description='true only with the robot on a stand'),
        DeclareLaunchArgument('robot_yaml', default_value='',
                              description='default: robot.yaml beside ~/learn2race_ws'),
        DeclareLaunchArgument('start_delay', default_value='8.0'),
        OpaqueFunction(function=launch_setup),
    ])
