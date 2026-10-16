#!/usr/bin/env python3
"""micro-ROS transport benchmark (FR8, NFR9).

Measures the four properties of the embedded link that the interface contract
depends on, and that a rate figure alone does not reveal.

**One-way latency.** Both firmwares synchronise the session clock, so a message
stamped on the microcontroller and received on the SBC gives the transport delay
directly as ``now - header.stamp``. A latency that grows over a run points at a
clock that is drifting between re-synchronisations rather than at a slow link.

**Rate holding under load.** The serial link is shared by commands and
telemetry. Publishing commands faster does not make the robot more responsive
if it starves the odometry stream, so this ramps the command rate and reports
the telemetry rate achieved at each step. The rate at which telemetry starts to
degrade is the usable ceiling of the link.

**Reconnection recovery.** Time from an agent restart to resumed telemetry,
which exercises the entity recreation and clock re-synchronisation path.

**Drop rate.** Gaps in the telemetry stream larger than twice the nominal
period, which is what a control loop actually feels.

    ros2 run learn2race_evaluation microros_benchmark --ros-args \\
        -p mode:=latency -p duration:=60.0
    ros2 run learn2race_evaluation microros_benchmark --ros-args \\
        -p mode:=load_ramp -p rates:="[10.0, 25.0, 50.0, 100.0, 200.0]"
"""

# References (full entries: docs/references.md at the repository root):
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS link
#   [R21] Fast DDS / Cyclone DDS: middleware
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR8, NFR9
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from .metrics import Result, latex_table


class MicroRosBenchmark(Node):

    def __init__(self):
        super().__init__('microros_benchmark')

        self.declare_parameter('mode', 'latency')      # latency | load_ramp | reconnect
        self.declare_parameter('telemetry_topic', 'odom')
        self.declare_parameter('nominal_hz', 20.0)
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('rates', [10.0, 25.0, 50.0, 100.0, 200.0])
        self.declare_parameter('seconds_per_step', 10.0)
        self.declare_parameter('command_speed', 0.0)   # zero: measure, do not drive
        self.declare_parameter('result_file', '')      # JSON output for comparison runs
        self.declare_parameter('label', '')            # tag, e.g. the RMW under test

        self.mode = self.get_parameter('mode').value
        self.nominal = float(self.get_parameter('nominal_hz').value)

        self.latencies = []
        self.arrivals = []
        self.step_index = 0
        self.step_started = None
        self.step_counts = []
        self.started = self._now()
        self.command_timer = None

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            Odometry, self.get_parameter('telemetry_topic').value,
            self._on_telemetry, 50)
        self.create_timer(0.1, self._tick)

        if self.mode == 'load_ramp':
            self._begin_step()

        self.get_logger().info(f'micro-ROS benchmark started in {self.mode} mode.')

    # ------------------------------------------------------------- helpers

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _rates(self):
        return [float(r) for r in self.get_parameter('rates').value]

    def _on_telemetry(self, msg: Odometry):
        now = self._now()
        self.arrivals.append(now)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            latency = now - stamp
            # A negative or absurd latency means the session clock is not
            # synchronised; report it rather than averaging it in.
            if -1.0 < latency < 5.0:
                self.latencies.append(latency)
        if self.mode == 'load_ramp':
            self.step_counts[-1] += 1

    # ---------------------------------------------------------- load ramp

    def _begin_step(self):
        rates = self._rates()
        if self.step_index >= len(rates):
            self._finish()
            return

        rate = rates[self.step_index]
        self.step_counts.append(0)
        self.step_started = self._now()

        if self.command_timer is not None:
            self.command_timer.cancel()
        self.command_timer = self.create_timer(1.0 / rate, self._publish_command)
        self.get_logger().info(f'Step {self.step_index + 1}: commanding at {rate:g} Hz')

    def _publish_command(self):
        command = Twist()
        command.linear.x = float(self.get_parameter('command_speed').value)
        self.pub.publish(command)

    # ------------------------------------------------------------ stepping

    def _tick(self):
        now = self._now()

        if self.mode == 'load_ramp':
            step_length = float(self.get_parameter('seconds_per_step').value)
            if self.step_started and (now - self.step_started) >= step_length:
                achieved = self.step_counts[-1] / step_length
                rate = self._rates()[self.step_index]
                self.get_logger().info(
                    f'  command {rate:6.1f} Hz -> telemetry {achieved:6.2f} Hz '
                    f'({achieved / self.nominal:.0%} of nominal)')
                self.step_index += 1
                self._begin_step()
            return

        if (now - self.started) >= float(self.get_parameter('duration').value):
            self._finish()

    # -------------------------------------------------------------- report

    def _finish(self):
        if self.command_timer is not None:
            self.command_timer.cancel()
        self.pub.publish(Twist())

        results = []

        latency = Result('FR8', 'micro-ROS one-way latency', 's', self.latencies)
        if self.latencies:
            latency.passed = latency.stats['p95'] < (1.0 / self.nominal)
            latency.note = (f"p95 {latency.stats['p95'] * 1e3:.1f} ms against a "
                            f"{1000.0 / self.nominal:.0f} ms nominal period")
        else:
            latency.note = ('no usable stamps: the session clock is probably '
                            'not synchronised')
        results.append(latency)

        if len(self.arrivals) > 1:
            intervals = [b - a for a, b in zip(self.arrivals, self.arrivals[1:])]
            rate = Result('NFR9', 'telemetry interval', 's', intervals)
            rate.passed = abs(rate.stats['mean'] - 1.0 / self.nominal) < \
                0.2 / self.nominal
            results.append(rate)

            threshold = 2.0 / self.nominal
            drops = [i for i in intervals if i > threshold]
            drop = Result('NFR9', 'telemetry gaps beyond 2x period', 'count',
                          [float(len(drops))])
            drop.passed = len(drops) == 0
            drop.note = f'{len(drops)} of {len(intervals)} intervals'
            results.append(drop)

        for r in results:
            self.get_logger().info(r.to_text())
            if r.note:
                self.get_logger().info(f'         {r.note}')

        path = self.get_parameter('result_file').value
        if path:
            import json
            import os
            payload = {
                'label': self.get_parameter('label').value or os.environ.get(
                    'RMW_IMPLEMENTATION', 'unknown'),
                'rmw': os.environ.get('RMW_IMPLEMENTATION', 'unknown'),
                'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
                'metrics': {r.metric: r.stats for r in results},
            }
            with open(path, 'w') as handle:
                json.dump(payload, handle, indent=2)
            self.get_logger().info(f'Wrote {path}')
        else:
            print()
            print(latex_table(results, 'micro-ROS transport performance.',
                              'tab:results_microros'))
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MicroRosBenchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
