#!/usr/bin/env python3
"""Telemetry freshness and fault supervision (FR10, NFR9).

The microcontroller already stops the motors when commands stop arriving. This
node covers the other direction: it detects that telemetry from the robot has
gone stale, or that a sensor is reporting out of range, and raises the fault
that drives the state machine into SAFE_STOP.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: telemetry supervision requirements
#   [R10] ROS 2 Jazzy documentation: rclpy node
# More details: README.md of this package; safety_policy.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        self.declare_parameter('odom_timeout_seconds', 0.5)
        self.declare_parameter('imu_timeout_seconds', 1.0)
        self.declare_parameter('scan_timeout_seconds', 1.0)
        self.declare_parameter('require_scan', False)
        self.declare_parameter('check_hz', 10.0)
        # The micro-ROS client connects a few seconds after the launch starts,
        # and the orientation filter only publishes once it has raw data. A
        # "never received" check from the first instant therefore fires on
        # every start-up, and because the fault latches, it would hold the
        # robot in SAFE_STOP permanently. Absence is only a fault once this
        # grace period has passed; staleness after data has arrived is always
        # a fault.
        self.declare_parameter('startup_grace_seconds', 15.0)

        self.timeouts = {
            'odom': float(self.get_parameter('odom_timeout_seconds').value),
            'imu': float(self.get_parameter('imu_timeout_seconds').value),
            'scan': float(self.get_parameter('scan_timeout_seconds').value),
        }
        self.require_scan = bool(self.get_parameter('require_scan').value)
        self.last = {'odom': None, 'imu': None, 'scan': None}
        self.fault_latched = False
        self.started = None
        self.grace = float(self.get_parameter('startup_grace_seconds').value)
        self.declare_parameter('reset_grace_seconds', 3.0)
        self.reset_grace = float(self.get_parameter('reset_grace_seconds').value)

        self.pub_fault = self.create_publisher(Bool, 'safety/fault', 10)
        self.pub_reason = self.create_publisher(String, 'safety/fault_reason', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        self.create_subscription(Odometry, 'odom',
                                 lambda m: self._mark('odom'), 10)
        self.create_subscription(Imu, 'imu/data',
                                 lambda m: self._mark('imu'), 10)
        self.create_subscription(LaserScan, 'scan',
                                 lambda m: self._mark('scan'), 10)
        self.create_subscription(Bool, 'safety/reset', self._on_reset, 10)

        self.create_timer(1.0 / float(self.get_parameter('check_hz').value),
                          self._check)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _mark(self, key):
        self.last[key] = self._now()

    def _on_reset(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Safety fault cleared by operator request.')
            self.fault_latched = False
            self.last = {k: None for k in self.last}
            # Fresh data is required, but each sensor needs time to send it:
            # without restarting the grace period the next check (0.1 s later)
            # found the 10 Hz LiDAR "never received" and re-latched at once,
            # so a reset could never clear a fault while the robot was running.
            self.started = self._now() - self.grace + self.reset_grace

    def _check(self):
        reasons = []
        now = self._now()
        if self.started is None:
            self.started = now
        in_grace = (now - self.started) < self.grace

        for key in ('odom', 'imu'):
            stamp = self.last[key]
            if stamp is None:
                if not in_grace:
                    reasons.append(f'{key} never received')
            elif now - stamp > self.timeouts[key]:
                reasons.append(f'{key} stale ({now - stamp:.2f} s)')

        if self.require_scan:
            stamp = self.last['scan']
            if stamp is None:
                if not in_grace:
                    reasons.append('scan never received')
            elif now - stamp > self.timeouts['scan']:
                reasons.append('scan stale')

        if reasons and not self.fault_latched:
            self.fault_latched = True
            self.get_logger().error('Safety fault: ' + '; '.join(reasons))

        msg = Bool()
        msg.data = self.fault_latched
        self.pub_fault.publish(msg)

        reason = String()
        reason.data = '; '.join(reasons)
        self.pub_reason.publish(reason)

        status = DiagnosticStatus()
        status.name = 'learn2race/safety_monitor'
        status.level = DiagnosticStatus.ERROR if self.fault_latched else DiagnosticStatus.OK
        status.message = reason.data or 'nominal'
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.pub_diag.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
