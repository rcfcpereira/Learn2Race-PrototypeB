#!/usr/bin/env python3
"""Assemble the Chapter 5 results tables from a campaign directory.

Runs every analysis over the bags of one campaign and writes a LaTeX fragment
that can be included directly, so that the numbers in the dissertation are the
numbers the tools produced.

    ros2 run learn2race_evaluation make_report --campaign runs/proto_a \\
        --out results_proto_a.tex
"""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: Chapter 5 results tables
#   [R16] rosbag2: bags
# More details: README.md of this package

import argparse
import glob
import os
import sys

from .analyze_timing import analyse as analyse_timing
from .analyze_tracking import stop_time, tracking_error
from .analyze_odometry import path_length
from .metrics import Result, latex_table


def collect(campaign: str, ground_truth: float):
    results = []

    straight = sorted(glob.glob(os.path.join(campaign, 'straight_*')))
    if straight:
        factors = [path_length(b) / ground_truth for b in straight]
        r = Result('FR2', 'odometry scale factor', '-', factors)
        r.passed = abs(r.stats['mean'] - 1.0) <= 0.05
        results.append(r)

    runs = sorted(glob.glob(os.path.join(campaign, 'run_*')))
    if runs:
        errors = []
        for bag in runs:
            errors.extend(tracking_error(bag, 0.5).values)
        r = Result('FR1', 'velocity tracking error', 'm/s', errors)
        r.passed = bool(errors) and r.stats['mean'] < 0.05
        results.append(r)

        stops = []
        for bag in runs:
            stops.extend(stop_time(bag, 0.01).values)
        r = Result('FR10', 'time to stop', 's', stops)
        r.passed = bool(stops) and r.stats['max'] < 1.0
        results.append(r)

        results.append(analyse_timing(runs[0], '/odom', 20.0, 0.20))

    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', required=True,
                        help='directory holding straight_* and run_* bags')
    parser.add_argument('--ground-truth', type=float, default=1.0)
    parser.add_argument('--out', default=None)
    parser.add_argument('--caption', default='Measured results.')
    parser.add_argument('--label', default='tab:results_campaign')
    args = parser.parse_args(argv)

    results = collect(args.campaign, args.ground_truth)
    if not results:
        print('No bags found. Expected straight_* and run_* under the campaign '
              'directory.', file=sys.stderr)
        return 1

    for r in results:
        print(r.to_text(), file=sys.stderr)

    table = latex_table(results, args.caption, args.label)
    if args.out:
        with open(args.out, 'w') as handle:
            handle.write(table)
        print(f'\nWrote {args.out}', file=sys.stderr)
    else:
        print(table)
    return 0


if __name__ == '__main__':
    sys.exit(main())
