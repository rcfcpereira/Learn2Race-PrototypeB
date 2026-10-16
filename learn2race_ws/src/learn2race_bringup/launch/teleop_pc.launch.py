"""Gamepad on a separate machine, driving the robot over the network.

Run this on the computer the gamepad is plugged into, while the robot runs
prototype_a.launch.py with allow_teleop:=true. Both machines must use the same
ROS_DOMAIN_ID and be on the same subnet.

    ros2 launch learn2race_bringup teleop_pc.launch.py

Nothing here publishes to cmd_vel directly. The commands go to teleop/cmd_vel,
and the multiplexer on the robot remains the only publisher of cmd_vel, so a
stale or duplicated command cannot reach the wheels behind its back.

Safety over a wireless link: the multiplexer drops a source that stops arriving
for source_timeout_seconds (0.5 s by default) and publishes zero instead, and
the microcontroller stops the motors after 500 ms without a command. A dropped
connection therefore stops the robot rather than leaving it running.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R19] joy / teleop_twist_joy: joy, teleop_twist_joy
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # SDL gamepad index (0 = first pad found). The Jazzy joy node ignores
        # /dev/input/js* paths; list pads with `ros2 run joy joy_enumerate_devices`.
        DeclareLaunchArgument('joy_device_id', default_value='0'),
        # Hold this button for the robot to move. Without it nothing is sent, so
        # a knock against the gamepad cannot drive the robot. 4 is the left
        # shoulder button on an Xbox pad; check yours with `ros2 topic echo /joy`.
        DeclareLaunchArgument('enable_button', default_value='4'),
        # A second button for a faster limit, useful once the robot is trusted.
        DeclareLaunchArgument('enable_turbo_button', default_value='5'),
        DeclareLaunchArgument('speed', default_value='0.25'),
        DeclareLaunchArgument('turbo_speed', default_value='0.6'),
        # Stick axes differ by OS and pad: the right stick's horizontal axis is
        # 3 on Linux (xpad) but 2 on Windows. Find yours by moving the stick
        # while watching `ros2 topic echo /joy --field axes`.
        DeclareLaunchArgument('linear_axis', default_value='1'),
        DeclareLaunchArgument('turn_axis', default_value='3'),
        DeclareLaunchArgument('turn_speed', default_value='1.5'),

        Node(package='joy', executable='joy_node', name='joy', output='screen',
             parameters=[{'device_name': '',
                          'device_id': LaunchConfiguration('joy_device_id'),
                          # Ignore tiny stick movements so a resting stick sends
                          # nothing, and repeat so a held stick keeps the
                          # multiplexer's source from timing out.
                          'deadzone': 0.10,
                          'autorepeat_rate': 20.0}]),

        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop',
             output='screen',
             parameters=[{'require_enable_button': True,
                          'enable_button': LaunchConfiguration('enable_button'),
                          'enable_turbo_button': LaunchConfiguration('enable_turbo_button'),
                          'axis_linear.x': LaunchConfiguration('linear_axis'),
                          'axis_angular.yaw': LaunchConfiguration('turn_axis'),
                          'scale_linear.x': LaunchConfiguration('speed'),
                          'scale_linear_turbo.x': LaunchConfiguration('turbo_speed'),
                          'scale_angular.yaw': LaunchConfiguration('turn_speed'),
                          'scale_angular_turbo.yaw': 2.5}],
             remappings=[('cmd_vel', 'teleop/cmd_vel')]),
    ])
