"""Odometry scale and heading drift analysis (FR2).

Two quantities, both derived from bags of deliberately simple manoeuvres:

* **Scale error** from a straight run over a measured ground-truth distance.
  A scale factor of 1.0 means reported distance matches reality. This is the
  measurement that exposed the reduction-ratio defect on Prototype A: with the
  ratio set to 50 against an actual 29, the factor came out near 0.58.

* **Heading drift** from N complete rotations in place. The robot should report
  N x 360 degrees; the residual is the drift, and it is dominated by the track
  width constant rather than by the wheel radius.

    ros2 run learn2race_evaluation analyze_odometry --bag straight_01 \\
        --mode scale --ground-truth 1.0
    ros2 run learn2race_evaluation analyze_odometry --bag spin_01 \\
        --mode heading --rotations 10
"""

# References (full entries: docs/references.md at the repository root):
#   [R42] Siegwart et al., Autonomous Mobile Robots: odometry error sources
#   [R43] Borenstein & Feng (1996) UMBmark: systematic odometry errors
#   [R16] rosbag2: bags
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR2
# More details: README.md of this package

import argparse
import math
import sys

from .bag_reader import BagReader
from .metrics import Result


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def path_length(bag_path: str, topic: str = '/odom') -> float:
    total = 0.0
    previous = None
    for _, msg in BagReader(bag_path).messages(topic):
        p = msg.pose.pose.position
        if previous is not None:
            total += math.hypot(p.x - previous[0], p.y - previous[1])
        previous = (p.x, p.y)
    return total


def unwrapped_yaw(bag_path: str, topic: str = '/odom') -> float:
    """Total rotation, accumulated so that wrapping does not lose turns."""
    total = 0.0
    previous = None
    for _, msg in BagReader(bag_path).messages(topic):
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        if previous is not None:
            delta = yaw - previous
            while delta > math.pi:
                delta -= 2.0 * math.pi
            while delta < -math.pi:
                delta += 2.0 * math.pi
            total += delta
        previous = yaw
    return total


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', action='append', required=True,
                        help='repeatable, one per repetition')
    parser.add_argument('--mode', choices=['scale', 'heading'], required=True)
    parser.add_argument('--topic', default='/odom')
    parser.add_argument('--ground-truth', type=float, default=1.0,
                        help='measured distance in metres (scale mode)')
    parser.add_argument('--rotations', type=int, default=10,
                        help='complete turns performed (heading mode)')
    parser.add_argument('--tolerance', type=float, default=0.05)
    args = parser.parse_args(argv)

    if args.mode == 'scale':
        factors = [path_length(b, args.topic) / args.ground_truth
                   for b in args.bag]
        result = Result('FR2', 'odometry scale factor', '-', factors,
                        acceptance=f'1.0 +/- {args.tolerance:.0%}')
        result.passed = abs(result.stats['mean'] - 1.0) <= args.tolerance
        if result.stats['n'] and result.stats['mean'] < 0.8:
            result.note = ('scale well below unity: check the reduction ratio '
                           'and the wheel radius before interpreting further')
    else:
        expected = args.rotations * 2.0 * math.pi
        drifts = [math.degrees(unwrapped_yaw(b, args.topic) - expected)
                  for b in args.bag]
        result = Result('FR2', 'heading drift', 'deg', drifts,
                        acceptance=f'within {args.tolerance * 100:.0f} deg')
        result.passed = abs(result.stats.get('mean', 0.0)) <= args.tolerance * 100
        result.note = 'drift is dominated by the track width constant'

    print(result.to_text())
    if result.note:
        print(f'{"":8s} {"":32s} {result.note}')
    print()
    print(result.to_latex_row())
    return 0 if result.passed else 1


if __name__ == '__main__':
    sys.exit(main())
