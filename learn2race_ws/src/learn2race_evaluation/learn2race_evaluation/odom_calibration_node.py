#!/usr/bin/env python3
"""Guided odometry calibration (FR2).

Drives a straight segment at low speed, then reports the correction that would
bring reported distance onto the measured ground truth. Rather than leaving the
operator to compute it, the node prints the corrected wheel radius and, for a
geared drivetrain, the reduction ratio implied by the error, since a scale error
of exactly the ratio of two plausible gear ratios usually means the constant is
wrong rather than the wheel worn.

    ros2 run learn2race_evaluation odom_calibration --ros-args \\
        -p distance_m:=1.0 -p speed:=0.15 -p wheel_radius:=0.0325
"""

# References (full entries: docs/references.md at the repository root):
#   [R42] Siegwart et al., Autonomous Mobile Robots: wheel radius from distance
#   [R62] CHP-20GP-180ABHL motor datasheet: gear ratio
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR2
# More details: README.md of this package

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomCalibration(Node):

    def __init__(self):
        super().__init__('odom_calibration')

        self.declare_parameter('distance_m', 1.0)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('gear_ratio', 29.0)
        self.declare_parameter('settle_seconds', 2.0)

        self.target = float(self.get_parameter('distance_m').value)
        self.speed = float(self.get_parameter('speed').value)

        self.start = None
        self.travelled = 0.0
        self.previous = None
        self.finished = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'Place the robot on a start line. Driving {self.target:.2f} m at '
            f'{self.speed:.2f} m/s. Measure the ACTUAL distance travelled.')

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.previous is not None:
            self.travelled += math.hypot(p.x - self.previous[0],
                                         p.y - self.previous[1])
        self.previous = (p.x, p.y)

    def _tick(self):
        if self.finished:
            return

        if self.travelled >= self.target:
            self.pub.publish(Twist())
            self.finished = True
            self._report()
            return

        command = Twist()
        command.linear.x = self.speed
        self.pub.publish(command)

    def _report(self):
        radius = float(self.get_parameter('wheel_radius').value)
        ratio = float(self.get_parameter('gear_ratio').value)

        self.get_logger().info('=' * 62)
        self.get_logger().info(f'Reported distance : {self.travelled:.4f} m')
        self.get_logger().info('Now measure the actual distance, then:')
        self.get_logger().info('')
        self.get_logger().info('  scale        = actual / reported')
        self.get_logger().info(f'  wheel_radius = {radius:.5f} * scale')
        self.get_logger().info(f'  gear_ratio   = {ratio:.1f} / scale')
        self.get_logger().info('')
        self.get_logger().info(
            'If the scale lands near a simple fraction of two standard gear '
            'ratios, suspect the ratio constant rather than the wheel. For '
            'example 29/50 = 0.58.')
        self.get_logger().info('=' * 62)
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
