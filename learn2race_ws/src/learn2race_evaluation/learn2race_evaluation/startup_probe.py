#!/usr/bin/env python3
"""Boot-to-ready measurement (NFR2, NFR5).

Time from power-on to a robot that can be commanded is the figure that governs
a teaching session: a fleet that takes three minutes to become usable loses a
meaningful fraction of a laboratory slot, and a robot that must be power-cycled
after a failed run pays that cost again.

The node reads the kernel uptime rather than its own start time, so the interval
reported includes firmware boot, kernel boot, systemd, the ROS graph and the
micro-ROS agent handshake, and not merely the part after the node launched.

Readiness is defined per topic, because the subsystems come up independently and
knowing which one dominates is what makes the number actionable.

    ros2 run learn2race_evaluation startup_probe --ros-args \\
        -p topics:="['/odom','/scan','/imu/data']" -p timeout:=120.0
"""

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: NFR2, NFR5
# More details: README.md of this package

import rclpy
from rclpy.node import Node


def kernel_uptime() -> float:
    """Seconds since kernel boot. Linux only; returns -1 elsewhere."""
    try:
        with open('/proc/uptime') as handle:
            return float(handle.readline().split()[0])
    except (OSError, ValueError):
        return -1.0


class StartupProbe(Node):

    def __init__(self):
        super().__init__('startup_probe')

        self.declare_parameter('topics', ['/odom', '/imu/data', '/scan'])
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('output', 'startup.csv')

        self.expected = list(self.get_parameter('topics').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.seen = {}
        self.node_start_uptime = kernel_uptime()
        self.started = self._now()

        if self.node_start_uptime < 0:
            self.get_logger().warn(
                'Kernel uptime unavailable; times will be relative to this '
                'node rather than to power-on.')

        self.get_logger().info(
            f'Probe running {self.node_start_uptime:.1f} s after boot. '
            f'Waiting for: {", ".join(self.expected)}')

        self.create_timer(0.1, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        info = dict(self.get_topic_names_and_types())

        for topic in self.expected:
            if topic in self.seen:
                continue
            if topic in info and self.count_publishers(topic) > 0:
                since_boot = kernel_uptime()
                self.seen[topic] = since_boot
                self.get_logger().info(
                    f'  {topic:20s} ready at {since_boot:6.1f} s after boot')

        if len(self.seen) == len(self.expected):
            self._report(complete=True)
        elif (self._now() - self.started) > self.timeout:
            self._report(complete=False)

    def _report(self, complete: bool):
        self.get_logger().info('=' * 62)
        if complete:
            ready = max(self.seen.values())
            self.get_logger().info(f'Boot to fully ready: {ready:.1f} s')
            slowest = max(self.seen, key=self.seen.get)
            self.get_logger().info(f'Last subsystem up  : {slowest}')
        else:
            missing = [t for t in self.expected if t not in self.seen]
            self.get_logger().error(
                f'Timed out. Never appeared: {", ".join(missing)}')

        path = self.get_parameter('output').value
        try:
            with open(path, 'w') as handle:
                handle.write('topic,seconds_after_boot\n')
                for topic, value in sorted(self.seen.items(), key=lambda kv: kv[1]):
                    handle.write(f'{topic},{value:.2f}\n')
            self.get_logger().info(f'Written to {path}')
        except OSError as exc:
            self.get_logger().error(f'Could not write {path}: {exc}')

        self.get_logger().info(
            'Compare with "systemd-analyze blame" to separate operating system '
            'boot from ROS graph startup.')
        self.get_logger().info('=' * 62)
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StartupProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
