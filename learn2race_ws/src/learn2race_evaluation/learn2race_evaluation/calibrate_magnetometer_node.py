#!/usr/bin/env python3
"""Magnetometer hard-iron calibration and axis check.

A magnetometer on a robot measures the Earth's field plus every field the robot
carries with it. Permanent magnets in the motors, the steel of the standoffs and
the current loops of the battery wiring all add a constant offset in the robot
frame, the hard-iron error, which is typically larger than the Earth's field
itself. Rotating the robot through a full turn traces a circle in the horizontal
field whose centre is displaced by exactly that offset, so the offset is the
midpoint of the extremes on each axis.

Two limitations are reported rather than hidden. The calibration removes the
constant offset, not the time-varying field from motor current, which changes
with load and cannot be calibrated out. And a heading estimate is only as good
as the local field, which indoors near steel structure may bear little relation
to north. The honest use of a compass on this platform is therefore as a
reference while stationary, not as a heading source during a run.

    ros2 run learn2race_evaluation calibrate_magnetometer
"""

# References (full entries: docs/references.md at the repository root):
#   [R49] NXP AN4246 magnetometer calibration: hard-iron offset
#   [R68] QMI8658C / AK09918 datasheets: AK09918
# More details: README.md of this package

import math
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from .calibration_store import CalibrationStore


class CalibrateMagnetometer(Node):

    def __init__(self):
        super().__init__('calibrate_magnetometer')
        self.declare_parameter('topic', 'imu/mag')
        self.declare_parameter('seconds', 30.0)
        self.declare_parameter('config_path', '')

        self.samples = []
        self.collecting = False
        path = self.get_parameter('config_path').value
        self.store = CalibrationStore(path) if path else CalibrationStore()

        self.create_subscription(MagneticField, self.get_parameter('topic').value,
                                 self._on_mag, 50)
        threading.Thread(target=self._run, daemon=True).start()

    def _on_mag(self, msg: MagneticField):
        if self.collecting:
            f = msg.magnetic_field
            self.samples.append((f.x * 1e6, f.y * 1e6, f.z * 1e6))   # microtesla

    def _run(self):
        seconds = float(self.get_parameter('seconds').value)
        print('\nMagnetometer hard-iron calibration.')
        print('Take the robot at least a metre from desks, radiators and steel.')
        print('Leave the motors UNPOWERED: their current adds a field that the')
        print('calibration cannot remove, and would corrupt the offset.\n')
        input(f'  Press Enter, then rotate the robot slowly through TWO full turns '
              f'over {seconds:.0f} s, keeping it level ')

        self.collecting = True
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            time.sleep(0.05)
        self.collecting = False

        if len(self.samples) < 50:
            print(f'\n  Only {len(self.samples)} samples. Is /imu/mag publishing?')
            rclpy.try_shutdown()
            return

        xs, ys, zs = zip(*self.samples)
        offset = [(max(a) + min(a)) / 2.0 for a in (xs, ys, zs)]
        span = [(max(a) - min(a)) for a in (xs, ys, zs)]

        # After removing the offset the horizontal field should be a circle
        # with a radius near the local Earth horizontal field, roughly 20 to 30
        # microtesla in Portugal. Much larger means residual interference;
        # strongly unequal x and y spans means soft-iron distortion.
        radius = (span[0] + span[1]) / 4.0
        ratio = span[0] / span[1] if span[1] > 0 else float('inf')

        print('\n  axis   offset (uT)   span (uT)')
        for axis, o, s in zip('xyz', offset, span):
            print(f'  {axis}      {o:+9.2f}   {s:9.2f}')
        print(f'\n  horizontal radius {radius:.1f} uT, x/y span ratio {ratio:.2f}')

        if not 15.0 <= radius <= 40.0:
            print('  The radius is outside the range expected for the Earth '
                  'field. Either interference remains, or the robot was not '
                  'rotated through a full turn.')
        if abs(ratio - 1.0) > 0.2:
            print('  x and y spans differ by more than 20 %: soft-iron '
                  'distortion is present, and an offset alone will leave a '
                  'heading error that varies with direction.')

        # Axis sanity: rotating anticlockwise, the field vector seen from the
        # robot must rotate CLOCKWISE. If it rotates the other way, one
        # horizontal axis sign is wrong relative to the body frame.
        headings = [math.atan2(y - offset[1], x - offset[0]) for x, y, _ in self.samples]
        turned = 0.0
        for a, b in zip(headings, headings[1:]):
            d = b - a
            turned += math.atan2(math.sin(d), math.cos(d))
        print(f'\n  apparent field rotation {math.degrees(turned):+.0f} deg '
              f'over the capture')
        print('  If the robot was turned anticlockwise, this should be negative. '
              'A positive value means a horizontal axis sign is inverted.')

        self.store.update('magnetometer', {
            'offset_x_ut': float(offset[0]),
            'offset_y_ut': float(offset[1]),
            'offset_z_ut': float(offset[2]),
            'horizontal_radius_ut': float(radius),
            'xy_span_ratio': float(ratio),
            'samples': len(self.samples),
        }, tool='calibrate_magnetometer')
        print(f'\nWritten to {self.store.save()}')
        print('Apply as MAG_OFFSET_*_UT build flags and reflash.')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateMagnetometer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
