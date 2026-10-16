#!/usr/bin/env python3
"""Command-timeout and safe-stop test (FR10, NFR1).

Brings the robot to a steady speed, then deliberately stops publishing commands
and measures how long the platform takes to reach standstill. This exercises the
firmware watchdog rather than a commanded deceleration, which is the behaviour
the requirement actually constrains: the robot must stop when the SBC goes away,
not only when it is asked to.

Runs the trial repeatedly and reports the distribution, because a watchdog that
usually fires quickly and occasionally does not is a different and more
dangerous object than one with a consistently longer delay.

    ros2 run learn2race_evaluation stop_test --ros-args -p repetitions:=10
"""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR10, NFR1 command timeout
#   [R61] TB6612FNG motor driver datasheet: short brake
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .metrics import Result


class StopTest(Node):

    def __init__(self):
        super().__init__('stop_test')

        self.declare_parameter('speed', 0.20)
        self.declare_parameter('run_up_seconds', 3.0)
        self.declare_parameter('standstill', 0.01)
        self.declare_parameter('repetitions', 5)
        self.declare_parameter('abort_seconds', 5.0)

        self.speed = float(self.get_parameter('speed').value)
        self.run_up = float(self.get_parameter('run_up_seconds').value)
        self.standstill = float(self.get_parameter('standstill').value)
        self.remaining = int(self.get_parameter('repetitions').value)

        self.phase = 'accelerate'
        self.phase_started = self._now()
        self.cut_at = None
        self.velocity = 0.0
        self.samples = []

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(0.02, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry):
        self.velocity = abs(msg.twist.twist.linear.x)

    def _tick(self):
        now = self._now()

        if self.phase == 'accelerate':
            command = Twist()
            command.linear.x = self.speed
            self.pub.publish(command)
            if now - self.phase_started >= self.run_up:
                # Stop publishing entirely. Do NOT send a zero twist: that
                # would test commanded braking, not the watchdog.
                self.phase = 'coast'
                self.cut_at = now
                self.get_logger().info('Command stream cut.')

        elif self.phase == 'coast':
            if self.velocity < self.standstill:
                elapsed = now - self.cut_at
                self.samples.append(elapsed)
                self.get_logger().info(
                    f'Stopped after {elapsed * 1e3:.0f} ms '
                    f'({len(self.samples)}/{len(self.samples) + self.remaining - 1})')
                self._next_repetition()
            elif now - self.cut_at > float(self.get_parameter('abort_seconds').value):
                self.get_logger().error(
                    'Robot did not stop within the abort window. Check the '
                    'firmware command timeout.')
                self.pub.publish(Twist())
                self._finish()

        elif self.phase == 'settle':
            self.pub.publish(Twist())
            if now - self.phase_started >= 2.0:
                self.phase = 'accelerate'
                self.phase_started = now

    def _next_repetition(self):
        self.remaining -= 1
        if self.remaining <= 0:
            self._finish()
        else:
            self.phase = 'settle'
            self.phase_started = self._now()

    def _finish(self):
        result = Result('FR10', 'time to stop after command loss', 's',
                        self.samples,
                        acceptance='within the configured command timeout')
        if self.samples:
            result.passed = result.stats['max'] < 1.0
        self.get_logger().info(result.to_text())
        print()
        print(result.to_latex_row())
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StopTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
