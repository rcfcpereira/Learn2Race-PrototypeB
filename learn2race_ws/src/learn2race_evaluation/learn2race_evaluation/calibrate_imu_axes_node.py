#!/usr/bin/env python3
"""Determine the IMU axis mapping empirically (REP-103 compliance).

ROS expects a body frame with x forward, y left and z up. An inertial unit
reports its own axes, defined relative to the chip package, and how those relate
to the robot depends on the chip orientation on the board and on how the board
is mounted. The Waveshare general driver documents the sensor's axes but not the
orientation of the sensor on the board, so the mapping cannot be looked up and
must be measured.

Measuring is preferable in any case, because the result also captures the
mounting, which no datasheet can know.

The procedure exploits the fact that a stationary accelerometer reads +1 g along
whichever axis points up. Presenting the robot in three known attitudes
therefore reveals which sensor axis corresponds to which robot axis, and with
which sign. The gyroscope is checked separately by rotating the robot
anticlockwise, which must produce positive yaw rate under the right-hand rule.

    ros2 run learn2race_evaluation calibrate_imu_axes
"""

# References (full entries: docs/references.md at the repository root):
#   [R12] REP 103, units and axes: x forward, y left, z up
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
AXES = ('x', 'y', 'z')


class CalibrateImuAxes(Node):

    def __init__(self):
        super().__init__('calibrate_imu_axes')

        self.declare_parameter('topic', 'imu/data')
        self.declare_parameter('samples', 200)
        self.declare_parameter('config_path', '')

        self.samples_needed = int(self.get_parameter('samples').value)
        self.collecting = False
        self.accel = {a: [] for a in AXES}
        self.gyro = {a: [] for a in AXES}

        path = self.get_parameter('config_path').value
        self.store = CalibrationStore(path) if path else CalibrationStore()

        self.create_subscription(Imu, self.get_parameter('topic').value,
                                 self._on_imu, 50)
        threading.Thread(target=self._run, daemon=True).start()

    def _on_imu(self, msg: Imu):
        if not self.collecting or len(self.accel['x']) >= self.samples_needed:
            return
        self.accel['x'].append(msg.linear_acceleration.x)
        self.accel['y'].append(msg.linear_acceleration.y)
        self.accel['z'].append(msg.linear_acceleration.z)
        self.gyro['x'].append(msg.angular_velocity.x)
        self.gyro['y'].append(msg.angular_velocity.y)
        self.gyro['z'].append(msg.angular_velocity.z)

    def _collect(self):
        self.accel = {a: [] for a in AXES}
        self.gyro = {a: [] for a in AXES}
        self.collecting = True
        while len(self.accel['x']) < self.samples_needed:
            time.sleep(0.1)
        self.collecting = False
        return ({a: summarise(self.accel[a])['mean'] for a in AXES},
                {a: summarise(self.gyro[a])['mean'] for a in AXES})

    @staticmethod
    def _dominant(readings):
        """Return (axis, sign) of the sensor axis reading closest to +/-1 g."""
        axis = max(AXES, key=lambda a: abs(readings[a]))
        return axis, (1 if readings[axis] > 0 else -1)

    def _run(self):
        print('\nIMU axis mapping.')
        print('ROS requires x forward, y left, z up (REP-103). This finds which')
        print('sensor axis corresponds to each, including the sign.\n')

        input('  1/3  Robot level on the floor, normal orientation. Enter ')
        level, _ = self._collect()
        up_axis, up_sign = self._dominant(level)
        print(f'       gravity appears on sensor {up_axis} '
              f'({level[up_axis]:+.2f}) -> robot Z is sensor '
              f'{"+" if up_sign > 0 else "-"}{up_axis}')

        input('  2/3  Tip the robot NOSE DOWN roughly 90 degrees. Enter ')
        nose, _ = self._collect()
        fwd_axis, down_sign = self._dominant(nose)
        # At rest an accelerometer reads +1 g on the axis pointing UP. Nose
        # down, robot +X points down, so the sensor axis along +X reads -1 g.
        fwd_sign = -down_sign
        print(f'       gravity appears on sensor {fwd_axis} '
              f'({nose[fwd_axis]:+.2f}) -> robot X is sensor '
              f'{"+" if fwd_sign > 0 else "-"}{fwd_axis}')

        input('  3/3  Roll the robot onto its RIGHT side roughly 90 degrees. Enter ')
        side, _ = self._collect()
        right_axis, up_reading_sign = self._dominant(side)
        # Right side down means robot +Y (left) points UP, so the sensor axis
        # along +Y reads +1 g: no sign change.
        left_sign = up_reading_sign
        print(f'       gravity appears on sensor {right_axis} '
              f'({side[right_axis]:+.2f}) -> robot Y is sensor '
              f'{"+" if left_sign > 0 else "-"}{right_axis}')

        chosen = {fwd_axis, right_axis, up_axis}
        if len(chosen) != 3:
            print('\n  Two robot axes resolved to the same sensor axis. A '
                  'position was probably not held square. Repeat, holding each '
                  'attitude steady and close to 90 degrees.')
            rclpy.try_shutdown()
            return

        # A rigidly mounted chip can only be rotated, never mirrored: the
        # mapping must have determinant +1. A mirror means one attitude was
        # not the one asked for (for example the left side instead of the right).
        index = {'x': 0, 'y': 1, 'z': 2}
        rows = []
        for axis, sign in ((fwd_axis, fwd_sign), (right_axis, left_sign), (up_axis, up_sign)):
            row = [0, 0, 0]
            row[index[axis]] = sign
            rows.append(row)
        det = (rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
               - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
               + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]))
        if det != 1:
            print('\n  The result is a mirror image, which no mounting can produce.')
            print('  One attitude was probably not as asked: step 2 must be NOSE')
            print('  DOWN and step 3 the robot\'s RIGHT side down (its left side up).')
            print('  Nothing was saved. Repeat the procedure.')
            rclpy.try_shutdown()
            return

        print('\n  Rotate the robot ANTICLOCKWISE (leftwards, seen from above)')
        input('       steadily while this samples. Press Enter, then rotate ')
        _, spin = self._collect()
        yaw_measured = spin[up_axis] * up_sign
        gyro_sign = 1 if yaw_measured > 0 else -1
        print(f'       yaw rate {yaw_measured:+.3f} rad/s')
        if gyro_sign < 0:
            print('       the gyroscope sign is inverted relative to the '
                  'right-hand rule and must be negated')

        mapping = {
            'x_source': fwd_axis, 'x_sign': int(fwd_sign),
            'y_source': right_axis, 'y_sign': int(left_sign),
            'z_source': up_axis, 'z_sign': int(up_sign),
            'gyro_sign': int(gyro_sign),
        }

        print('\n' + '=' * 60)
        print('  ROS axis      <-  sensor axis')
        print(f'  x (forward)   <-  {"+" if fwd_sign > 0 else "-"}{fwd_axis}')
        print(f'  y (left)      <-  {"+" if left_sign > 0 else "-"}{right_axis}')
        print(f'  z (up)        <-  {"+" if up_sign > 0 else "-"}{up_axis}')
        print('=' * 60)

        identity = (mapping['x_source'] == 'x' and mapping['x_sign'] == 1 and
                    mapping['y_source'] == 'y' and mapping['y_sign'] == 1 and
                    mapping['z_source'] == 'z' and mapping['z_sign'] == 1)
        if identity:
            print('\n  The sensor is already aligned with the body frame; no '
                  'remapping is required.')
        else:
            print('\n  Apply this in firmware, not on the SBC: the message '
                  'should be correct where it is created, so that every '
                  'consumer sees a REP-103 frame. Build flags for Prototype A:')
            print(f'    -D IMU_X_SOURCE={fwd_axis.upper()} '
                  f'-D IMU_X_SIGN={fwd_sign}')
            print(f'    -D IMU_Y_SOURCE={right_axis.upper()} '
                  f'-D IMU_Y_SIGN={left_sign}')
            print(f'    -D IMU_Z_SOURCE={up_axis.upper()} '
                  f'-D IMU_Z_SIGN={up_sign}')
            print('  Prototype B (lsm6ds3_imu parameters in prototype_b.yaml):')
            print(f'    axis_x_source: {index[fwd_axis]}  axis_x_sign: {fwd_sign}')
            print(f'    axis_y_source: {index[right_axis]}  axis_y_sign: {left_sign}')
            print(f'    axis_z_source: {index[up_axis]}  axis_z_sign: {up_sign}')
            print(f'    gyro_sign: {gyro_sign}')

        self.store.update('imu_axes', mapping, tool='calibrate_imu_axes')
        print(f'\nWritten to {self.store.save()}')
        print('Re-run the gyroscope bias calibration after changing the '
              'mapping: the bias belongs to the remapped axes.')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateImuAxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
