"""Interface rate and jitter analysis (FR8, NFR9).

Verifies the claim made by the SBC-MCU interface table: that odometry is
published at 20 Hz and commands dispatched at 50 Hz. The configured period is
visible in the firmware source; what this measures is the period achieved under
load, which is the part that can fail.

Jitter is reported as the standard deviation of the inter-message interval and
as the 95th percentile, because a control loop is harmed more by an occasional
long gap than by a slightly wrong mean.

    ros2 run learn2race_evaluation analyze_timing --bag run_01 \\
        --topic /odom --expected-hz 20
"""

# References (full entries: docs/references.md at the repository root):
#   [R14] micro-ROS / micro_ros_arduino: SBC-MCU interface rates
#   [R16] rosbag2: bags
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR8, NFR9
# More details: README.md of this package

import argparse
import sys

from .bag_reader import BagReader
from .metrics import Result, latex_table


def analyse(bag_path: str, topic: str, expected_hz: float,
            tolerance: float) -> Result:
    stamps = BagReader(bag_path).timestamps(topic)
    if len(stamps) < 2:
        return Result('FR8', f'{topic} interval', 's', [],
                      note='fewer than two messages')

    intervals = [b - a for a, b in zip(stamps, stamps[1:])]
    result = Result(
        requirement='FR8',
        metric=f'{topic} interval',
        unit='s',
        values=intervals,
        acceptance=f'{expected_hz:g} Hz within {tolerance:.0%}')

    expected = 1.0 / expected_hz
    result.passed = abs(result.stats['mean'] - expected) <= tolerance * expected
    result.note = (f"achieved {1.0 / result.stats['mean']:.2f} Hz, "
                   f"p95 interval {result.stats['p95'] * 1e3:.1f} ms")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--topic', action='append', required=True,
                        help='repeatable: --topic /odom --topic /cmd_vel')
    parser.add_argument('--expected-hz', type=float, action='append', required=True)
    parser.add_argument('--tolerance', type=float, default=0.20)
    parser.add_argument('--latex', action='store_true',
                        help='emit a LaTeX table instead of plain text')
    args = parser.parse_args(argv)

    if len(args.topic) != len(args.expected_hz):
        parser.error('--topic and --expected-hz must be given the same number of times')

    results = [analyse(args.bag, topic, hz, args.tolerance)
               for topic, hz in zip(args.topic, args.expected_hz)]

    if args.latex:
        print(latex_table(results,
                          'Measured interface rate and jitter.',
                          'tab:results_timing'))
    else:
        for r in results:
            print(r.to_text())
            if r.note:
                print(f'{"":8s} {"":32s} {r.note}')

    return 0 if all(r.passed for r in results) else 1


if __name__ == '__main__':
    sys.exit(main())
