#!/usr/bin/env python3
"""Inertial unit calibration.

Two corrections are produced, and they are separate problems.

**Gyroscope bias.** A rate gyroscope reads a small non-zero value when
stationary, and that offset integrates directly into heading error: an
uncorrected bias of 0.01 rad/s becomes 34 degrees of drift in a minute. The bias
is temperature-dependent, so the robot is left powered for a warm-up period
before sampling rather than measured from cold.

**Accelerometer scale and offset.** Measured by presenting each axis to gravity
in turn, six positions in total. With the axis up the reading should be +g and
with it down it should be -g; the midpoint of the two gives the offset and half
their difference gives the scale. This separates the two errors, which a single
level measurement cannot do.

    ros2 run learn2race_evaluation calibrate_imu --ros-args -p mode:=gyro
    ros2 run learn2race_evaluation calibrate_imu --ros-args -p mode:=accel
"""

# References (full entries: docs/references.md at the repository root):
#   [R12] REP 103, units and axes: units
#   [R42] Siegwart et al., Autonomous Mobile Robots: gyro bias and accelerometer offset
# More details: README.md of this package

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .calibration_store import CalibrationStore
from .metrics import summarise

GRAVITY = 9.80665

POSITIONS = [
    ('x', +1, 'X axis pointing UP'),
    ('x', -1, 'X axis pointing DOWN'),
    ('y', +1, 'Y axis pointing UP'),
    ('y', -1, 'Y axis pointing DOWN'),
    ('z', +1, 'Z axis pointing UP (robot level, normal orientation)'),
    ('z', -1, 'Z axis pointing DOWN (robot inverted)'),
]


class CalibrateImu(Node):

    def __init__(self):
        super().__init__('calibrate_imu')

        self.declare_parameter('mode', 'gyro')          # gyro | accel
        self.declare_parameter('topic', 'imu/data')
        self.declare_parameter('samples', 500)
        self.declare_parameter('warmup_seconds', 60.0)
        self.declare_parameter('config_path', '')

        self.samples_needed = int(self.get_parameter('samples').value)
        self.collecting = False
        self.accel = {'x': [], 'y': [], 'z': []}
        self.gyro = {'x': [], 'y': [], 'z': []}

        path = self.get_parameter('config_path').value
        self.store = CalibrationStore(path) if path else CalibrationStore()

        self.create_subscription(Imu, self.get_parameter('topic').value,
                                 self._on_imu, 50)
        threading.Thread(target=self._run, daemon=True).start()

    def _on_imu(self, msg: Imu):
        if not self.collecting:
            return
        if len(self.accel['x']) >= self.samples_needed:
            return
        self.accel['x'].append(msg.linear_acceleration.x)
        self.accel['y'].append(msg.linear_acceleration.y)
        self.accel['z'].append(msg.linear_acceleration.z)
        self.gyro['x'].append(msg.angular_velocity.x)
        self.gyro['y'].append(msg.angular_velocity.y)
        self.gyro['z'].append(msg.angular_velocity.z)

    def _collect(self):
        self.accel = {'x': [], 'y': [], 'z': []}
        self.gyro = {'x': [], 'y': [], 'z': []}
        self.collecting = True
        while len(self.accel['x']) < self.samples_needed:
            time.sleep(0.1)
        self.collecting = False

    def _run(self):
        if self.get_parameter('mode').value == 'gyro':
            self._calibrate_gyro()
        else:
            self._calibrate_accel()

    def _calibrate_gyro(self):
        warmup = float(self.get_parameter('warmup_seconds').value)
        print(f'\nGyroscope bias calibration.')
        print(f'Leave the robot completely still and powered for {warmup:.0f} s '
              f'so the sensor reaches a stable temperature, then press Enter.')
        print('Do not touch the robot or the surface it stands on.\n')
        input('  Press Enter when the warm-up is done ')

        self._collect()

        bias = {axis: summarise(self.gyro[axis])['mean'] for axis in 'xyz'}
        noise = {axis: summarise(self.gyro[axis])['sd'] for axis in 'xyz'}

        print('\n  axis      bias (rad/s)   noise sd (rad/s)   drift (deg/min)')
        for axis in 'xyz':
            drift = bias[axis] * 60.0 * 57.2958
            print(f'  {axis}      {bias[axis]:+12.6f}   {noise[axis]:14.6f}   '
                  f'{drift:+12.2f}')

        worst = max(abs(bias[a]) * 60.0 * 57.2958 for a in 'xyz')
        if worst > 5.0:
            print(f'\n  Uncorrected, the worst axis drifts {worst:.1f} deg/min. '
                  f'Applying this bias is not optional for heading estimation.')

        self.store.update('imu_gyro', {
            'bias_x': float(bias['x']),
            'bias_y': float(bias['y']),
            'bias_z': float(bias['z']),
            'noise_sd_x': float(noise['x']),
            'noise_sd_y': float(noise['y']),
            'noise_sd_z': float(noise['z']),
            'samples': self.samples_needed,
        }, tool='calibrate_imu')
        print(f'\nWritten to {self.store.save()}')
        rclpy.try_shutdown()

    def _calibrate_accel(self):
        print('\nAccelerometer scale and offset calibration, six positions.')
        print('Hold each position steady and square; a few degrees of tilt '
              'changes the reading more than the error being measured.\n')

        readings = {}
        for axis, sign, description in POSITIONS:
            input(f'  {description}, then press Enter ')
            self._collect()
            readings[(axis, sign)] = summarise(self.accel[axis])['mean']
            print(f'    {axis} reads {readings[(axis, sign)]:+7.3f} m/s^2 '
                  f'(expected {sign * GRAVITY:+7.3f})')

        result = {}
        print('\n  axis   offset (m/s^2)   scale')
        for axis in 'xyz':
            up, down = readings[(axis, +1)], readings[(axis, -1)]
            offset = (up + down) / 2.0          # midpoint: the zero error
            scale = (up - down) / (2.0 * GRAVITY)  # half-span over one g
            result[f'offset_{axis}'] = float(offset)
            result[f'scale_{axis}'] = float(scale)
            print(f'  {axis}      {offset:+13.4f}   {scale:.5f}')

        worst = max(abs(1.0 - result[f'scale_{a}']) for a in 'xyz')
        if worst > 0.05:
            print(f'\n  Scale error of {worst:.1%} on the worst axis suggests '
                  f'the full-scale range configured in firmware does not match '
                  f'the conversion factor used to publish the message.')

        self.store.update('imu_accel', result, tool='calibrate_imu')
        print(f'\nWritten to {self.store.save()}')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
