#!/usr/bin/env python3
"""Reflective line array calibration.

The array reports raw counts, and those counts depend on surface reflectivity,
sensor height and ambient light as much as on whether a sensor is over the line.
Comparing raw counts against a fixed threshold therefore works in one room and
fails in the next. Calibration records, per channel, the value seen over the
brightest surface and the darkest, so subsequent readings can be normalised onto
a common zero-to-one scale.

Per-channel bounds are necessary rather than a single pair: the emitters and
detectors vary between channels, and the array is rarely exactly parallel to the
surface, so the outer channels usually sit at a different height from the inner
ones.

    ros2 run learn2race_evaluation calibrate_line_array
"""

# References (full entries: docs/references.md at the repository root):
#   [R80] QYF-750 line-sensor array: QYF-750 array
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS7830
# More details: README.md of this package

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from .calibration_store import CalibrationStore


class CalibrateLineArray(Node):

    def __init__(self):
        super().__init__('calibrate_line_array')

        self.declare_parameter('topic', 'line_array/raw')
        self.declare_parameter('channels', 8)
        self.declare_parameter('sweep_seconds', 15.0)
        self.declare_parameter('config_path', '')

        self.channels = int(self.get_parameter('channels').value)
        self.minimum = [10**6] * self.channels
        self.maximum = [-10**6] * self.channels
        self.samples = 0
        self.collecting = False

        path = self.get_parameter('config_path').value
        self.store = CalibrationStore(path) if path else CalibrationStore()

        self.create_subscription(Int32MultiArray,
                                 self.get_parameter('topic').value,
                                 self._on_array, 50)
        threading.Thread(target=self._run, daemon=True).start()

    def _on_array(self, msg: Int32MultiArray):
        if not self.collecting:
            return
        self.samples += 1
        for i, value in enumerate(list(msg.data)[:self.channels]):
            self.minimum[i] = min(self.minimum[i], value)
            self.maximum[i] = max(self.maximum[i], value)

    def _run(self):
        seconds = float(self.get_parameter('sweep_seconds').value)
        print('\nLine array calibration.')
        print(f'Sweep the array slowly across the line and back for {seconds:.0f} s, '
              'so that every channel sees both the line and the surface either '
              'side of it. Keep the array at its normal ride height.\n')
        input('  Press Enter to start the sweep ')

        self.collecting = True
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        while (self.get_clock().now().nanoseconds * 1e-9) < end:
            time.sleep(0.05)
        self.collecting = False

        if self.samples == 0:
            print('\n  No data received. Is the line array node running?')
            rclpy.try_shutdown()
            return

        print(f'\n  {self.samples} samples.\n')
        print('  channel    min    max   span')
        weak = []
        for i in range(self.channels):
            span = self.maximum[i] - self.minimum[i]
            flag = ''
            if span < 30:
                flag = '  <- little contrast'
                weak.append(i)
            print(f'  {i:5d}    {self.minimum[i]:5d}  {self.maximum[i]:5d}  '
                  f'{span:5d}{flag}')

        if weak:
            print(f'\n  Channels {weak} saw almost no contrast. Either the '
                  f'sweep did not carry the line under them, or those emitters '
                  f'sit at the wrong height, or they have failed. Re-run before '
                  f'accepting these bounds.')

        self.store.update('line_array', {
            'calibration_min': [int(v) for v in self.minimum],
            'calibration_max': [int(v) for v in self.maximum],
            'channels': self.channels,
            'samples': self.samples,
        }, tool='calibrate_line_array')
        print(f'\nWritten to {self.store.save()}')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLineArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
