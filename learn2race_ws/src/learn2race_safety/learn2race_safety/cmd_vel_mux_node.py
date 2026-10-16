#!/usr/bin/env python3
"""Command arbitration with an arming lock.

Both competitions prohibit teleoperation and remote control during a run, and
require that the robot does not move between the countdown button and the start
signal. This node is the single publisher of the drive command: it forwards one
selected source and drops everything else.

Priority, highest first:
    1. competition/cmd_vel   (state machine; always allowed, carries the stop)
    2. autonomy/cmd_vel      (track following; allowed only when armed)
    3. teleop/cmd_vel        (gamepad; blocked whenever competition_mode is set)

When disarmed the node publishes zero at a fixed rate rather than nothing, so
the microcontroller watchdog sees a live but stationary command stream.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR10 command timeout and safe stop; drive-source priorities
#   [R10] ROS 2 Jazzy documentation: rclpy node
# More details: README.md of this package; safety_policy.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class CmdVelMux(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('competition_mode', True)
        self.declare_parameter('publish_hz', 25.0)
        self.declare_parameter('source_timeout_seconds', 0.5)

        self.competition_mode = bool(self.get_parameter('competition_mode').value)
        self.timeout = float(self.get_parameter('source_timeout_seconds').value)

        self.armed = False
        self.sources = {'competition': (None, 0.0),
                        'autonomy': (None, 0.0),
                        'teleop': (None, 0.0)}

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'competition/cmd_vel',
                                 lambda m: self._store('competition', m), 10)
        self.create_subscription(Twist, 'autonomy/cmd_vel',
                                 lambda m: self._store('autonomy', m), 10)
        self.create_subscription(Twist, 'teleop/cmd_vel',
                                 lambda m: self._store('teleop', m), 10)
        self.create_subscription(Bool, 'safety/arm', self._on_arm, 10)

        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _store(self, name, msg):
        self.sources[name] = (msg, self._now())

    def _on_arm(self, msg: Bool):
        if msg.data != self.armed:
            self.get_logger().info(f'Arming state -> {msg.data}')
        self.armed = msg.data

    def _fresh(self, name):
        msg, stamp = self.sources[name]
        if msg is None:
            return None
        return msg if (self._now() - stamp) <= self.timeout else None

    def _tick(self):
        selected = self._fresh('competition')

        if selected is None and self.armed:
            selected = self._fresh('autonomy')

        if selected is None and not self.competition_mode:
            selected = self._fresh('teleop')

        self.pub.publish(selected if selected is not None else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
