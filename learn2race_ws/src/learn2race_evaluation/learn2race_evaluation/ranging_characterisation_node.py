#!/usr/bin/env python3
"""Guided characterisation of a ranging sensor against ground truth.

Reported range is not the same as true distance. Every ranging technology on
these robots has a different error structure, and the useful result is a
characterisation rather than a single accuracy figure:

* The VL6180X and TOF050C are time-of-flight, roughly linear within their band,
  but sensitive to target reflectivity and to ambient infrared.
* The Sharp GP2Y0A21 is triangulating and strongly non-linear, and its response
  folds back below about 10 cm so that one output voltage corresponds to two
  distances. A calibration that ignores the fold produces confident nonsense at
  close range.
* The ultrasonic ranger is nearly linear but fails on absorbent or steeply
  angled surfaces, where the failure is a missing echo rather than a wrong one.

The node prompts for each ground-truth distance, collects a sample at rest, and
reports bias and precision per distance, then emits a table. Measuring precision
separately from bias matters: a sensor with a consistent offset can be
corrected, whereas one with a large spread cannot.

    ros2 run learn2race_evaluation ranging_characterisation --ros-args \\
        -p topic:=/vl6180/range_0 \\
        -p distances:="[0.05,0.10,0.15,0.20,0.30,0.40]"
"""

# References (full entries: docs/references.md at the repository root):
#   [R72] VL6180X datasheet, TOF050C module: VL6180X / TOF050C
#   [R73] Sharp GP2Y0A21YK0F datasheet: Sharp GP2Y0A21
#   [R74] Seeed Grove module wiki: Grove ultrasonic
# More details: README.md of this package

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .metrics import Result, summarise


class RangingCharacterisation(Node):

    def __init__(self):
        super().__init__('ranging_characterisation')

        self.declare_parameter('topic', 'tof/range')
        self.declare_parameter('distances', [0.05, 0.10, 0.15, 0.20, 0.30, 0.40])
        self.declare_parameter('samples_per_point', 50)
        self.declare_parameter('label', 'ranging sensor')

        self.distances = [float(d) for d in self.get_parameter('distances').value]
        self.samples_needed = int(self.get_parameter('samples_per_point').value)

        self.collecting = False
        self.samples = []
        self.table = []

        self.create_subscription(Range, self.get_parameter('topic').value,
                                 self._on_range, 50)

        threading.Thread(target=self._run, daemon=True).start()

    def _on_range(self, msg: Range):
        if self.collecting and len(self.samples) < self.samples_needed:
            self.samples.append(float(msg.range))

    def _run(self):
        label = self.get_parameter('label').value
        print(f'\nCharacterising {label} on '
              f'{self.get_parameter("topic").value}.')
        print('Place a flat, matte target square to the sensor at each '
              'distance. Reflectivity affects time-of-flight sensors, so use '
              'the same target throughout.\n')

        for truth in self.distances:
            input(f'  Target at {truth * 100:5.1f} cm, then press Enter ')
            self.samples = []
            self.collecting = True

            while len(self.samples) < self.samples_needed:
                time.sleep(0.1)
            self.collecting = False

            stats = summarise(self.samples)
            bias = stats['mean'] - truth
            print(f'    reported {stats["mean"] * 100:6.2f} cm   '
                  f'bias {bias * 100:+6.2f} cm   '
                  f'sd {stats["sd"] * 1000:5.2f} mm   '
                  f'n={stats["n"]}')
            self.table.append((truth, stats, bias))

        self._report()

    def _report(self):
        biases = [abs(b) for _, _, b in self.table]
        spreads = [s['sd'] for _, s, _ in self.table]

        accuracy = Result('FR2', f'{self.get_parameter("label").value} bias', 'm',
                          biases)
        precision = Result('FR2', f'{self.get_parameter("label").value} precision',
                           'm', spreads)

        print()
        print(accuracy.to_text())
        print(precision.to_text())

        # A bias that grows with distance is a scale error and is correctable
        # by a single factor; a constant offset is an origin error. Reporting
        # which one it is tells the reader how to fix it.
        if len(self.table) >= 2:
            first, last = self.table[0], self.table[-1]
            growth = abs(last[2]) - abs(first[2])
            if abs(growth) > 2.0 * max(spreads):
                print(f'\nBias grows by {growth * 100:+.2f} cm across the range: '
                      f'this is a scale error, correctable by one factor.')
            else:
                print(f'\nBias is approximately constant: this is an origin '
                      f'offset, correctable by subtraction.')

        print('\n% --- table rows for Chapter 5 ---')
        for truth, stats, bias in self.table:
            print(f'\t\t\t\t\t\\num{{{truth * 100:.1f}}} & '
                  f'\\num{{{stats["mean"] * 100:.2f}}} & '
                  f'\\num{{{bias * 100:+.2f}}} & '
                  f'\\num{{{stats["sd"] * 1000:.2f}}} & {stats["n"]} \\\\')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RangingCharacterisation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
