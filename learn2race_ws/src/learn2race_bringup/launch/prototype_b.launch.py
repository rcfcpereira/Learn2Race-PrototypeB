"""Full bringup for Prototype B (teaching and mapping platform).

Competition nodes are off by default; enable them for classroom demonstration
of the state machine with competition:=true. Prototype B is built for teaching
and mapping, not for competition.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent
#   [R46] Madgwick (2010) orientation filter: imu_filter_madgwick
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar driver
# More details: README.md of this package

import os

import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def sensor_frame_publishers(params_path):
    """base_link -> sensor static transforms from the yaml sensor_frames section.

    Cameras get an extra fixed <side>_camera_link -> <side>_camera_optical_frame
    rotation (optical: z forward, x right, y down), since camera_ros stamps its
    images in the optical frame.
    """
    with open(params_path) as f:
        section = (yaml.safe_load(f) or {}).get('sensor_frames') or {}
    actions = []
    if not section.get('measured', False):
        actions.append(LogInfo(msg='[prototype_b] sensor_frames are not measured yet: '
                                   'sensor positions are placeholders (prototype_b.yaml).'))

    def static(parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=f'tf_{child}', arguments=[
                        '--x', str(x), '--y', str(y), '--z', str(z),
                        '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                        '--frame-id', parent, '--child-frame-id', child])

    for name, pose in (section.get('frames') or {}).items():
        p = {k: float(pose.get(k, 0.0)) for k in ('x', 'y', 'z', 'yaw')}
        if name in ('left_camera', 'right_camera'):
            link = f'{name}_link'
            actions.append(static('base_link', link, **p))
            actions.append(static(link, f'{name}_optical_frame',
                                  roll=-math.pi / 2, yaw=-math.pi / 2))
        else:
            actions.append(static('base_link', name, **p))
    return actions


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('learn2race_bringup'),
        'config', 'prototype_b.yaml')

    competition = LaunchConfiguration('competition')
    teleop = LaunchConfiguration('teleop')
    serial_port = LaunchConfiguration('serial_port')
    lidar_port = LaunchConfiguration('lidar_port')

    frames = sensor_frame_publishers(os.path.join(
        get_package_share_directory('learn2race_bringup'), 'config', 'sensor_frames_b.yaml'))
    return LaunchDescription(frames + [
        DeclareLaunchArgument('competition', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='true'),
        # The DDSM HAT wires the ESP32 UART both to its USB bridge and to the
        # Pi header (GPIO14/15). With enable_uart=1 the Pi's TX drives the
        # ESP32's RX line, so replies sent over USB (/dev/mcu) never arrive:
        # the ESP32 pings forever and no session forms. The header UART works;
        # keep USB for flashing only (with the Pi disconnected).
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        # Must equal ROS_SERIAL_BAUD in the firmware platformio.ini. A mismatch
        # does not produce an error: the agent simply never sees the client.
        DeclareLaunchArgument('agent_baud', default_value='921600'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar'),
        # No device answers at the TOF050C's 0x52 on the bus or on any mux
        # channel; only the three VL6180X (channels 0-2) are fitted. Enable
        # once a TOF050C is wired, or the node fails every cycle.
        DeclareLaunchArgument('tof050c', default_value='false'),
        # The pack-monitor analogue inputs are not wired on this robot; an
        # unconnected input reads noise (5.9 V "flat pack" errors). Enable
        # once the power module is connected and calibrated.
        DeclareLaunchArgument('battery', default_value='false'),
        # Grove port of the chainable LED: D5 = clock GPIO5, data GPIO6.
        DeclareLaunchArgument('led_clock_pin', default_value='5'),
        DeclareLaunchArgument('led_data_pin', default_value='6'),
        # Wheel odometry owns odom -> base_link. Set false when another source
        # does (rtabmap_stereo_b.launch.py visual odometry): a frame may have
        # only one parent in TF. /odom from the ESP32 is still published.
        DeclareLaunchArgument('wheel_odom_tf', default_value='true'),
        # ekf:=true: robot_localization fuses the wheel speed and yaw rate with the gyro yaw rate
        # (learn2race_localization/config/ekf_b_imu.yaml) and owns odom -> base_link instead of the
        # wheel-odometry transform. Measured better for fast, turning autonomous runs (15 Oct 2026).
        DeclareLaunchArgument('ekf', default_value='false'),

        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', serial_port,
                        '-b', LaunchConfiguration('agent_baud')]),

        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen',
             parameters=[{'product_name': 'LDLiDAR_LD06',
                          'topic_name': 'scan',
                          'frame_id': 'base_laser',
                          'port_name': lidar_port,
                          'port_baudrate': 230400}]),

        Node(package='learn2race_hardware', executable='vl6180_tof_node',
             name='vl6180_tof', parameters=[params]),
        Node(package='learn2race_hardware', executable='tof050c_node',
             name='tof050c', parameters=[params],
             condition=IfCondition(LaunchConfiguration('tof050c'))),
        Node(package='learn2race_hardware', executable='lsm6ds3_imu_node',
             name='lsm6ds3_imu', parameters=[params]),
        Node(package='learn2race_hardware', executable='i2c_display_node',
                name='i2c_display', parameters=[{
                     'i2c_bus': 1,
                     'i2c_address': 0x3c,
                     'width': 128,
                     'height': 32,
                }]),
        Node(package='learn2race_hardware', executable='battery_monitor',
             name='battery_monitor', parameters=[params],
             condition=IfCondition(LaunchConfiguration('battery'))),
        # Status LED: Grove Chainable RGB LED (P9813) on two Pi GPIOs, in place of the
        # WS2813 stick that moved to Prototype A on 2026-10-01. It shows leds/set_color.
        Node(package='learn2race_hardware', executable='chainable_led_node',
             name='chainable_led', parameters=[{
                 'clock_pin': ParameterValue(LaunchConfiguration('led_clock_pin'), value_type=int),
                 'data_pin': ParameterValue(LaunchConfiguration('led_data_pin'), value_type=int),
                 'led_count': 1,
             }]),

        Node(package='learn2race_odometry', executable='odom_tf_broadcaster_node',
             name='odometry', parameters=[params],
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('wheel_odom_tf'), "' == 'true' and '",
                 LaunchConfiguration('ekf'), "' != 'true'"]))),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(get_package_share_directory('learn2race_localization'),
                                      'config', 'ekf_b_imu.yaml')],
             condition=IfCondition(LaunchConfiguration('ekf'))),
        # Orientation: fuse the LSM6DS3's raw rate and acceleration
        # (imu/data_raw) into imu/data with orientation set, as on Prototype
        # A. The Grove board has no magnetometer, so use_mag stays false and
        # yaw drifts slowly; odometry and the LiDAR correct it downstream.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter', output='screen',
             parameters=[{'use_mag': False,
                          'publish_tf': False,
                          'world_frame': 'enu',
                          'gain': 0.1,
                          'fixed_frame': 'odom'}]),
        Node(package='learn2race_safety', executable='safety_monitor',
             name='safety_monitor', parameters=[params]),
        Node(package='learn2race_safety', executable='cmd_vel_mux',
             name='cmd_vel_mux', parameters=[params]),

        Node(package='joy', executable='joy_node', name='joy',
             condition=IfCondition(teleop)),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop', condition=IfCondition(teleop),
             remappings=[('cmd_vel', 'teleop/cmd_vel')]),

        Node(package='learn2race_competition', executable='competition_fsm',
             name='competition_fsm', condition=IfCondition(competition),
             parameters=[params]),
        Node(package='learn2race_competition', executable='led_signalling',
             name='led_signalling', condition=IfCondition(competition),
             parameters=[params]),
    ])
