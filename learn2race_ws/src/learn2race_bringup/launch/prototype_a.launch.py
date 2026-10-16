"""Full bringup for Prototype A (competition + education).

    ros2 launch learn2race_bringup prototype_a.launch.py mode:=trackbotgp
    ros2 launch learn2race_bringup prototype_a.launch.py mode:=dragster
    ros2 launch learn2race_bringup prototype_a.launch.py competition:=false
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent
#   [R46] Madgwick (2010) orientation filter: imu_filter_madgwick
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar driver
#   [R1]  TrackBotGP rules 2026: competition modes
# More details: README.md of this package

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('learn2race_bringup'),
        'config', 'prototype_a.yaml')

    mode = LaunchConfiguration('mode')
    competition = LaunchConfiguration('competition')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='trackbotgp'),
        DeclareLaunchArgument('competition', default_value='true'),
        DeclareLaunchArgument('line_array', default_value='false'),
        # Manual driving with a gamepad. The competition rules forbid
        # teleoperation during a run, so the multiplexer blocks it by default.
        #
        #   teleop:=true        run the gamepad ON THE ROBOT and lift the block
        #   allow_teleop:=true  lift the block only, for a gamepad on another
        #                       machine running teleop_pc.launch.py
        #
        # Never use either in competition.
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('allow_teleop', default_value='false'),
        DeclareLaunchArgument('rgb_stick', default_value='false'),
        # Stable names created by scripts/setup_udev.sh. ttyUSB0/ttyUSB1 swap
        # depending on enumeration order when both the board and the LiDAR are
        # connected, so the agent can end up opening the LiDAR. Override with
        # serial_port:=/dev/ttyUSB0 if the udev rule is not installed.
        DeclareLaunchArgument('serial_port', default_value='/dev/mcu'),
        # Must equal ROS_SERIAL_BAUD in the firmware platformio.ini. A mismatch
        # does not produce an error: the agent simply never sees the client.
        DeclareLaunchArgument('agent_baud', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),

        # --- embedded layer -------------------------------------------------
        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', serial_port,
                        '-b', LaunchConfiguration('agent_baud')]),

        # --- sensing --------------------------------------------------------
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen',
             parameters=[{'product_name': 'LDLiDAR_LD06',
                          'topic_name': 'scan',
                          'frame_id': 'base_laser',
                          'port_name': lidar_port,
                          'port_baudrate': 230400}]),
        # The reflective line array is a planned extension and is not fitted
        # yet; without the ADS7830 on the bus the node fails with a remote I/O
        # error. Enable with line_array:=true once the hardware is installed.
        Node(package='learn2race_hardware', executable='line_array_node',
             name='line_array', parameters=[params],
             condition=IfCondition(LaunchConfiguration('line_array'))),
        # No ltr303_node here. On Prototype A the LTR-303 sits on the driver
        # board I2C bus, which the Pi cannot reach, and the firmware already
        # publishes light/visible_plus_ir_raw over micro-ROS. A Pi-side node
        # finds no device (I/O error) and would duplicate the topic if it did.

        # --- orientation ----------------------------------------------------
        # The firmware publishes raw rate and acceleration on imu/data_raw and
        # the field on imu/mag. The Madgwick filter fuses them into imu/data
        # with the orientation set, which is what robot_localization, RViz and
        # the safety monitor consume.
        #
        # use_mag is FALSE by default. The compass sits centimetres from two
        # brushed motors whose field changes with load, which cannot be
        # calibrated out; fusing it would inject a load-dependent heading error
        # during every run. Enable it only for stationary heading tests.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter', output='screen',
             parameters=[{'use_mag': False,
                          'publish_tf': False,
                          'world_frame': 'enu',
                          'gain': 0.1,
                          'fixed_frame': 'odom'}]),

        # --- estimation -----------------------------------------------------
        Node(package='learn2race_odometry', executable='odom_tf_broadcaster_node',
             name='odometry', parameters=[params]),

        # --- preview --------------------------------------------------------
        # No standalone preview here. The camera line follower serves the
        # preview itself at http://<robot>:8080/ when started with
        # camera_line_follower.launch.py, reusing the frames it already has.
        # This launch file starts no camera, so a preview here would show
        # nothing.

        # --- safety ---------------------------------------------------------
        Node(package='learn2race_safety', executable='safety_monitor',
             name='safety_monitor', parameters=[params]),
        Node(package='learn2race_safety', executable='cmd_vel_mux',
             name='cmd_vel_mux',
             parameters=[params, {'competition_mode': ParameterValue(
                 PythonExpression(["'", LaunchConfiguration('teleop'), "' != 'true' and '",
                                   LaunchConfiguration('allow_teleop'), "' != 'true'"]),
                 value_type=bool)}]),

        # Gamepad. Requires ros-jazzy-joy and ros-jazzy-teleop-twist-joy; if
        # either is missing, launch aborts before anything starts, so install
        # them before using teleop:=true.
        #
        # Publishes to teleop/cmd_vel, never straight to cmd_vel: the
        # multiplexer is the single publisher of cmd_vel, and a second publisher
        # there would alternate with the multiplexer's zero stream, making the
        # robot lurch between the command and a stop.
        Node(package='joy', executable='joy_node', name='joy',
             condition=IfCondition(LaunchConfiguration('teleop'))),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop',
             condition=IfCondition(LaunchConfiguration('teleop')),
             parameters=[{'require_enable_button': True,
                          'enable_button': 4,
                          'axis_linear.x': 1,
                          'axis_angular.yaw': 3,
                          'scale_linear.x': 0.25,
                          'scale_angular.yaw': 1.5}],
             remappings=[('cmd_vel', 'teleop/cmd_vel')]),

        # --- competition ----------------------------------------------------
        Node(package='learn2race_competition', executable='competition_fsm',
             name='competition_fsm', condition=IfCondition(competition),
             parameters=[params, {'mode': mode}]),
        Node(package='learn2race_competition', executable='lap_counter',
             name='lap_counter', condition=IfCondition(competition),
             parameters=[params]),
        Node(package='learn2race_competition', executable='led_signalling',
             name='led_signalling', condition=IfCondition(competition),
             parameters=[params]),
        # Prototype A has no RGB LED hardware fitted (see the bill of
        # materials), so the stick driver is off unless explicitly enabled.
        Node(package='learn2race_hardware', executable='rgb_stick_node',
             name='rgb_stick', condition=IfCondition(LaunchConfiguration('rgb_stick'))),
    ])
