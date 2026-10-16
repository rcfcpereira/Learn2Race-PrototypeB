"""Velocity tracking and stopping analysis (FR1, FR10).

Pairs each commanded velocity with the measured velocity nearest in time and
reports the steady-state error, ignoring a settling window after each change of
command so that transients are not counted as tracking error.

Stop time is measured from the last command message to the first sample below
the standstill threshold, which is the quantity the command-timeout requirement
constrains.

    ros2 run learn2race_evaluation analyze_tracking --bag run_01
"""

# References (full entries: docs/references.md at the repository root):
#   [R40] Astrom & Murray, Feedback Systems (PID): step response, steady-state error
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR1, FR10
# More details: README.md of this package

import argparse
import sys

from .bag_reader import BagReader
from .metrics import Result


def _nearest(samples, t):
    return min(samples, key=lambda s: abs(s[0] - t)) if samples else None


def tracking_error(bag_path: str, settle_s: float) -> Result:
    reader = BagReader(bag_path)
    commands = [(ts * 1e-9, m.linear.x) for ts, m in reader.messages('/cmd_vel')]
    measured = [(ts * 1e-9, m.twist.twist.linear.x)
                for ts, m in BagReader(bag_path).messages('/odom')]

    errors = []
    for index, (t, target) in enumerate(commands):
        if target == 0.0:
            continue
        if index > 0 and commands[index - 1][1] != target:
            continue                      # skip the sample at a step change
        sample = _nearest([s for s in measured if s[0] >= t + settle_s], t)
        if sample is not None:
            errors.append(abs(sample[1] - target))

    result = Result('FR1', 'velocity tracking error', 'm/s', errors)
    return result


def stop_time(bag_path: str, standstill: float) -> Result:
    reader = BagReader(bag_path)
    commands = [ts * 1e-9 for ts, _ in reader.messages('/cmd_vel')]
    measured = [(ts * 1e-9, abs(m.twist.twist.linear.x))
                for ts, m in BagReader(bag_path).messages('/odom')]

    if not commands or not measured:
        return Result('FR10', 'time to stop', 's', [], note='insufficient data')

    last_command = commands[-1]
    after = [s for s in measured if s[0] > last_command]
    stopped = next((t for t, v in after if v < standstill), None)

    values = [stopped - last_command] if stopped is not None else []
    result = Result('FR10', 'time to stop', 's', values,
                    acceptance='command timeout plus deceleration')
    if not values:
        result.note = 'robot never reached standstill in this recording'
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', action='append', required=True)
    parser.add_argument('--settle', type=float, default=0.5)
    parser.add_argument('--standstill', type=float, default=0.01)
    args = parser.parse_args(argv)

    for bag in args.bag:
        print(f'--- {bag}')
        for result in (tracking_error(bag, args.settle),
                       stop_time(bag, args.standstill)):
            print(result.to_text())
            if result.note:
                print(f'{"":8s} {"":32s} {result.note}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
