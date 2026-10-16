"""Launch only the Xbox joystick and teleop on a remote PC.

This launch assumes the remote PC can see the Pi over the ROS 2 network. It
does not start the micro-ROS bridge or any hardware-specific nodes. Commands go
to teleop/cmd_vel so the robot's multiplexer stays the only cmd_vel publisher;
publishing to cmd_vel directly alternates with its zero stream. Start the robot
with teleop:=false so its own teleop node does not also react to this /joy.
"""

# References (full entries: docs/references.md at the repository root):
#   [R15] ROS 2 launch: launch
#   [R19] joy / teleop_twist_joy: joy
# More details: README.md of this package

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # SDL gamepad index; the Jazzy joy node ignores /dev/input/js* paths.
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/teleop/cmd_vel"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "device_id": LaunchConfiguration("joy_device_id"),
                        "deadzone": 0.08,
                        "autorepeat_rate": 30.0,
                        "coalesce_interval_ms": 1,
                    }
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[
                    {
                        "require_enable_button": True,
                        "enable_button": 4,
                        "enable_turbo_button": 5,
                        "axis_linear": {"x": 1},
                        "scale_linear": {"x": 0.25},
                        "scale_linear_turbo": {"x": 0.60},
                        "axis_angular": {"yaw": 3},
                        "scale_angular": {"yaw": 1.20},
                        "scale_angular_turbo": {"yaw": 2.50},
                        "publish_stamped_twist": False,
                    }
                ],
                remappings=[
                    ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
                ],
            ),
        ]
    )
