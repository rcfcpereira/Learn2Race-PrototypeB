#!/usr/bin/env python3
"""First-line sensor diagnostic for both prototypes (NFR1, NFR5).

Checks every sensor a prototype is expected to have, in one command, and reports
which are publishing, at what rate, and whether their values are physically
plausible. This is the tool that lets a student distinguish a hardware fault
from a software defect without tracing wires, which the systematic review found
to be the failure students most often get wrong.

Three things are checked per sensor, because they fail differently. A topic that
never appears means the driver did not start or the device is absent from the
bus. A topic that appears at the wrong rate usually means bus contention or a
misconfigured device. A topic at the right rate carrying implausible values
means the device is present and talking but miswired, saturated, or
uncalibrated.

    ros2 run learn2race_evaluation sensor_check --ros-args -p prototype:=a
"""

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: NFR1, NFR5
# More details: README.md of this package

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (BatteryState, Illuminance, Image, Imu, LaserScan,
                             MagneticField, Range)
from std_msgs.msg import Float32MultiArray, UInt32

GRAVITY = 9.80665


class SensorCheck(Node):

    # (topic, type, expected_hz, plausibility check or None)
    COMMON = [
        ('odom', Odometry, 20.0, 'odom'),
        ('imu/data', Imu, 50.0, 'imu'),
        ('scan', LaserScan, 10.0, 'scan'),
        ('battery_state', BatteryState, 1.0, 'battery'),
    ]
    PROTOTYPE_A = [
        ('imu/data_raw', Imu, 50.0, 'imu'),
        ('imu/mag', MagneticField, 20.0, 'mag'),
        ('light/visible_plus_ir_raw', UInt32, 10.0, None),
        ('line_array/normalised', Float32MultiArray, 100.0, 'array'),
        ('image_raw', Image, 15.0, None),
    ]
    PROTOTYPE_B = [
        ('tof/range', Range, 20.0, 'range'),
        # The three VL6180X behind the PCA9548A publish these (vl6180_tof_node 'topics').
        ('/tof/front', Range, 20.0, 'range'),
        ('/tof/left', Range, 20.0, 'range'),
        ('/tof/right', Range, 20.0, 'range'),
        ('dragster/light_ratio', UInt32, 10.0, None),
    ]

    def __init__(self):
        super().__init__('sensor_check')

        self.declare_parameter('prototype', 'a')
        self.declare_parameter('window_seconds', 10.0)
        self.declare_parameter('rate_tolerance', 0.4)

        prototype = str(self.get_parameter('prototype').value).lower()
        expected = self.COMMON + (self.PROTOTYPE_A if prototype == 'a'
                                  else self.PROTOTYPE_B)

        self.tolerance = float(self.get_parameter('rate_tolerance').value)
        self.records = {}

        for topic, msg_type, hz, check in expected:
            self.records[topic] = {'count': 0, 'hz': hz, 'check': check,
                                   'issues': set(), 'last': None,
                                   'values': [], 'bad': 0}
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_message(t, msg), 10)

        self.started = self._now()
        self.create_timer(float(self.get_parameter('window_seconds').value),
                          self._report)

        self.get_logger().info(
            f'Checking {len(expected)} sensors on Prototype '
            f'{prototype.upper()} over '
            f'{self.get_parameter("window_seconds").value:.0f} s. '
            f'Leave the robot stationary and level.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_message(self, topic, msg):
        record = self.records[topic]
        record['count'] += 1
        record['last'] = msg

        check = record['check']
        if check == 'imu':
            magnitude = math.sqrt(msg.linear_acceleration.x ** 2 +
                                  msg.linear_acceleration.y ** 2 +
                                  msg.linear_acceleration.z ** 2)
            # Judged over the whole window, not per sample: one corrupted read
            # among hundreds must not hide a sensor that is otherwise correct,
            # and must not be reported as though it were typical.
            record['values'].append(magnitude)
            if abs(magnitude - GRAVITY) > 1.0:
                record['bad'] += 1
            record.setdefault('gyro', []).append(
                max(abs(msg.angular_velocity.x), abs(msg.angular_velocity.y),
                    abs(msg.angular_velocity.z)))

        elif check == 'scan':
            finite = [r for r in msg.ranges
                      if msg.range_min < r < msg.range_max and math.isfinite(r)]
            if not finite:
                record['issues'].add('every return invalid')
            elif len(finite) < 0.3 * len(msg.ranges):
                record['issues'].add(
                    f'only {100 * len(finite) / len(msg.ranges):.0f}% valid returns')

        elif check == 'range':
            if not math.isfinite(msg.range):
                record['issues'].add('non-finite range')
            elif msg.range <= msg.min_range or msg.range >= msg.max_range:
                record['issues'].add(f'{msg.range:.3f} m is outside the sensor band')

        elif check == 'array':
            values = list(msg.data)
            if values and max(values) - min(values) < 0.05:
                # A flat array means the sensor sees no contrast: either it is
                # off the track or the calibration bounds are wrong.
                record['issues'].add('no contrast across channels')

        elif check == 'odom':
            speed = abs(msg.twist.twist.linear.x)
            if speed > 0.02:
                record['issues'].add(f'reports {speed:.3f} m/s while stationary')

        elif check == 'mag':
            f = msg.magnetic_field
            magnitude = math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2) * 1e6
            record['values'].append(magnitude)
            if not 15.0 < magnitude < 120.0:
                record['bad'] += 1

        elif check == 'battery':
            if msg.voltage < 1.0:
                record['issues'].add(f'{msg.voltage:.2f} V is implausible')

    def _report(self):
        elapsed = self._now() - self.started
        failures = 0

        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'{"topic":30s} {"rate":>12s} {"expected":>10s}  status')
        self.get_logger().info('-' * 70)

        for topic, record in self.records.items():
            achieved = record['count'] / elapsed
            expected = record['hz']

            values = sorted(record['values'])
            if values:
                median = values[len(values) // 2]
                fraction = record['bad'] / len(values)
                if record['check'] == 'imu':
                    if abs(median - GRAVITY) > 1.0:
                        record['issues'].add(
                            f'median |a| = {median:.2f} m/s^2, expected 9.81')
                    gyro = sorted(record.get('gyro', []))
                    if gyro and gyro[len(gyro) // 2] > 0.05:
                        g = gyro[len(gyro) // 2]
                        record['issues'].add(
                            f'median gyro rate {g:.3f} rad/s while still '
                            f'({g * 57.3:.1f} deg/s, {g * 57.3 * 60:.0f} deg/min '
                            f'of heading drift): run calibrate_imu mode:=gyro')
                    if fraction > 0.0 and abs(median - GRAVITY) <= 1.0:
                        record['issues'].add(
                            f'median |a| = {median:.2f} is correct, but '
                            f'{fraction:.0%} of samples are implausible: '
                            f'intermittent bad reads on the bus')
                elif record['check'] == 'mag':
                    if median > 120.0:
                        record['issues'].add(
                            f'median |B| = {median:.0f} uT: large hard-iron offset '
                            f'from motor magnets and steel nearby; run '
                            f'calibrate_magnetometer')
                    elif median < 15.0:
                        record['issues'].add(
                            f'median |B| = {median:.1f} uT: near zero, the compass '
                            f'data is not reaching the message')

            if record['count'] == 0:
                status = 'ABSENT'
                failures += 1
            elif abs(achieved - expected) > self.tolerance * expected:
                status = 'RATE'
                failures += 1
            elif record['issues']:
                status = 'IMPLAUSIBLE'
                failures += 1
            else:
                status = 'ok'

            self.get_logger().info(
                f'{topic:30s} {achieved:9.2f} Hz {expected:8.1f} Hz  {status}')
            for issue in sorted(record['issues']):
                self.get_logger().info(f'{"":30s} {"":23s}  - {issue}')

        self.get_logger().info('-' * 70)
        if failures:
            self.get_logger().error(
                f'{failures} of {len(self.records)} sensors need attention.')
            self.get_logger().info(
                'ABSENT: driver not started or device missing from the bus. '
                'RATE: bus contention or misconfiguration. IMPLAUSIBLE: the '
                'device is talking but miswired, saturated or uncalibrated.')
        else:
            self.get_logger().info('All sensors nominal.')
        self.get_logger().info('=' * 70)
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
