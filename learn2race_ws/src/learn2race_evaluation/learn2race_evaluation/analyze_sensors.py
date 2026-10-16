"""Offline sensor analysis from recorded bags (FR2, NFR1).

Two analyses that need a recording rather than a live stream, because both
depend on statistics over time rather than on an instantaneous value.

**IMU static characterisation.** With the robot stationary and level, the
accelerometer should read exactly one g and the gyroscope exactly zero. What it
actually reads gives the bias, which is subtracted in the estimator, and the
standard deviation, which sets how much the fusion should trust it. Reporting
these makes the covariance values in the localisation configuration defensible
rather than arbitrary.

**LiDAR quality.** Rate, points per revolution, the proportion of returns that
are valid, and range accuracy against a wall at a measured distance. The valid
return proportion is the figure that matters in a teaching venue: polished
floors and glass partitions both produce dropouts, and a scan that looks fine
on a plot can be missing a third of its returns.

    ros2 run learn2race_evaluation analyze_sensors --bag static_01 --mode imu
    ros2 run learn2race_evaluation analyze_sensors --bag wall_01 --mode scan \\
        --wall-distance 1.0
"""

# References (full entries: docs/references.md at the repository root):
#   [R16] rosbag2: bags
#   [R12] REP 103, units and axes: IMU units
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR2, NFR1
# More details: README.md of this package

import argparse
import math
import sys

from .bag_reader import BagReader
from .metrics import Result, summarise

GRAVITY = 9.80665


def imu_static(bag_path: str, topic: str = '/imu/data'):
    accel = {'x': [], 'y': [], 'z': []}
    gyro = {'x': [], 'y': [], 'z': []}

    for _, msg in BagReader(bag_path).messages(topic):
        accel['x'].append(msg.linear_acceleration.x)
        accel['y'].append(msg.linear_acceleration.y)
        accel['z'].append(msg.linear_acceleration.z)
        gyro['x'].append(msg.angular_velocity.x)
        gyro['y'].append(msg.angular_velocity.y)
        gyro['z'].append(msg.angular_velocity.z)

    if not accel['x']:
        print('No IMU messages in this bag.', file=sys.stderr)
        return []

    magnitudes = [math.sqrt(x * x + y * y + z * z)
                  for x, y, z in zip(accel['x'], accel['y'], accel['z'])]

    results = []
    scale = Result('FR2', 'accelerometer magnitude', 'm/s^2', magnitudes)
    scale.passed = abs(scale.stats['mean'] - GRAVITY) < 0.5
    scale.note = (f"expected {GRAVITY:.3f}; error "
                  f"{scale.stats['mean'] - GRAVITY:+.3f} m/s^2")
    results.append(scale)

    for axis in ('x', 'y', 'z'):
        stats = summarise(gyro[axis])
        bias = Result('FR2', f'gyro {axis} bias', 'rad/s', [stats['mean']])
        bias.passed = abs(stats['mean']) < 0.02
        bias.note = f"noise sd {stats['sd']:.5f} rad/s"
        results.append(bias)

    print('\n% Suggested covariance diagonal, from the measured noise:')
    for axis in ('x', 'y', 'z'):
        variance = summarise(gyro[axis])['sd'] ** 2
        print(f'%   angular_velocity_covariance[{axis}] = {variance:.3e}')

    return results


def scan_quality(bag_path: str, wall_distance: float, topic: str = '/scan'):
    stamps, valid_fractions, wall_errors, point_counts = [], [], [], []

    for stamp, msg in BagReader(bag_path).messages(topic):
        stamps.append(stamp * 1e-9)
        point_counts.append(len(msg.ranges))

        finite = [r for r in msg.ranges
                  if math.isfinite(r) and msg.range_min < r < msg.range_max]
        valid_fractions.append(len(finite) / max(1, len(msg.ranges)))

        if wall_distance > 0 and finite:
            # The forward beam is the middle of the array for a scan centred
            # on the robot's x axis.
            forward = msg.ranges[len(msg.ranges) // 2]
            if math.isfinite(forward):
                wall_errors.append(forward - wall_distance)

    if not stamps:
        print('No scan messages in this bag.', file=sys.stderr)
        return []

    results = []
    if len(stamps) > 1:
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]
        rate = Result('FR7', 'scan interval', 's', intervals)
        rate.note = f"achieved {1.0 / summarise(intervals)['mean']:.2f} Hz"
        results.append(rate)

    points = Result('FR7', 'points per scan', 'count',
                    [float(c) for c in point_counts])
    results.append(points)

    valid = Result('NFR1', 'valid return fraction', '-', valid_fractions)
    valid.passed = summarise(valid_fractions)['mean'] > 0.7
    valid.note = ('dropouts on polished floors and glass are expected; '
                  'below 0.7 the venue is the problem, not the sensor')
    results.append(valid)

    if wall_errors:
        accuracy = Result('FR7', 'range error against wall', 'm', wall_errors)
        accuracy.passed = abs(summarise(wall_errors)['mean']) < 0.05
        results.append(accuracy)

    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--mode', choices=['imu', 'scan'], required=True)
    parser.add_argument('--topic', default=None)
    parser.add_argument('--wall-distance', type=float, default=0.0)
    args = parser.parse_args(argv)

    if args.mode == 'imu':
        results = imu_static(args.bag, args.topic or '/imu/data')
    else:
        results = scan_quality(args.bag, args.wall_distance,
                               args.topic or '/scan')

    for result in results:
        print(result.to_text())
        if result.note:
            print(f'{"":8s} {"":32s} {result.note}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
