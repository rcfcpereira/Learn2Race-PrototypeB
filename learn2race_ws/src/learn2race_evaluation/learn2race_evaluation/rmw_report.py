#!/usr/bin/env python3
"""Compare middleware implementations from a set of benchmark runs.

Reads the JSON files written by ``microros_benchmark`` with ``result_file`` set,
one per middleware, and emits a comparison table.

The question this answers is specific to the platform rather than general: which
middleware leaves the most headroom on a Raspberry Pi that is already running a
camera pipeline, and which degrades most gracefully once a teaching laboratory
fills the wireless network. A middleware that wins a benchmark on a workstation
is not necessarily the one to deploy on a fleet of single-board computers.

    ros2 run learn2race_evaluation rmw_report --results runs/rmw/*.json \\
        --cpu runs/rmw/cpu.csv --out results_rmw.tex
"""

# References (full entries: docs/references.md at the repository root):
#   [R21] Fast DDS / Cyclone DDS: Fast DDS vs Cyclone DDS
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent
# More details: README.md of this package

import argparse
import csv
import json
import sys

SHORT = {
    'rmw_fastrtps_cpp': 'Fast DDS',
    'rmw_cyclonedds_cpp': 'Cyclone DDS',
    'rmw_zenoh_cpp': 'Zenoh',
    'rmw_connextdds': 'Connext DDS',
}


def load(paths):
    runs = []
    for path in paths:
        with open(path) as handle:
            runs.append(json.load(handle))
    return runs


def load_cpu(path):
    """Mean CPU percentage per label, from the sampler CSV."""
    if not path:
        return {}
    totals = {}
    try:
        with open(path) as handle:
            for row in csv.DictReader(handle):
                totals.setdefault(row['label'], []).append(float(row['cpu_percent']))
    except (OSError, KeyError, ValueError) as exc:
        print(f'Could not read CPU samples: {exc}', file=sys.stderr)
        return {}
    return {k: sum(v) / len(v) for k, v in totals.items() if v}


def _ms(stats, key):
    if not stats or not stats.get('n'):
        return '---'
    return f"\\num{{{stats[key] * 1e3:.2f}}}"


def build_rows(runs, cpu):
    rows = []
    for run in runs:
        name = SHORT.get(run.get('rmw', ''), run.get('label', 'unknown'))
        metrics = run.get('metrics', {})

        latency = metrics.get('micro-ROS one-way latency', {})
        interval = metrics.get('telemetry interval', {})
        gaps = metrics.get('telemetry gaps beyond 2x period', {})

        if interval.get('n') and interval.get('mean'):
            achieved = f"\\num{{{1.0 / interval['mean']:.2f}}}"
        else:
            achieved = '---'

        gap_count = str(int(gaps['mean'])) if gaps.get('n') else '---'

        label = run.get('label', '')
        cpu_cell = (f"\\num{{{cpu[label]:.1f}}}" if label in cpu
                    else '\\dataneeded{measure}')

        rows.append(
            f"\t\t\t\t\t{name} & {_ms(latency, 'mean')} & {_ms(latency, 'p95')} & "
            f"{achieved} & {_ms(interval, 'sd')} & {gap_count} & {cpu_cell} \\\\")
    return rows


def latex(runs, cpu):
    body = '\n\t\t\t\t\t\\addlinespace\n'.join(build_rows(runs, cpu))
    header = (r"\textbf{Middleware} & \textbf{Latency mean (ms)} & "
              r"\textbf{Latency p95 (ms)} & \textbf{Rate (Hz)} & "
              r"\textbf{Jitter SD (ms)} & \textbf{Gaps} & \textbf{CPU (\%)} \\")
    spec = (r"@{}X >{\raggedleft\arraybackslash}p{1.5cm} "
            r">{\raggedleft\arraybackslash}p{1.5cm} "
            r">{\raggedleft\arraybackslash}p{1.4cm} "
            r">{\raggedleft\arraybackslash}p{1.5cm} "
            r">{\raggedleft\arraybackslash}p{1.1cm} "
            r">{\raggedleft\arraybackslash}p{1.3cm}@{}")

    return (
        "\t\t\t\\begin{table}[hbt]\n"
        "\t\t\t\t\\centering\n"
        "\t\t\t\t\\small\n"
        "\t\t\t\t\\caption{Middleware comparison on the target hardware.}\n"
        "\t\t\t\t\\label{tab:results_rmw}\n"
        f"\t\t\t\t\\begin{{tabularx}}{{\\linewidth}}{{{spec}}}\n"
        "\t\t\t\t\t\\toprule\n"
        "\t\t\t\t\t\\rowcolor[rgb]{0.863,0.902,0.945}\n"
        f"\t\t\t\t\t{header}\n"
        "\t\t\t\t\t\\midrule\n"
        f"{body}\n"
        "\t\t\t\t\t\\bottomrule\n"
        "\t\t\t\t\\end{tabularx}\n"
        "\t\t\t\\end{table}\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', nargs='+', required=True)
    parser.add_argument('--cpu', default=None)
    parser.add_argument('--out', default=None)
    args = parser.parse_args(argv)

    runs = load(args.results)
    if not runs:
        print('No result files given.', file=sys.stderr)
        return 1

    for run in runs:
        metrics = run.get('metrics', {})
        latency = metrics.get('micro-ROS one-way latency', {})
        if latency.get('n'):
            print(f"{run.get('label', '?'):22s} "
                  f"latency mean {latency['mean'] * 1e3:6.2f} ms  "
                  f"p95 {latency['p95'] * 1e3:6.2f} ms", file=sys.stderr)

    table = latex(runs, load_cpu(args.cpu))
    if args.out:
        with open(args.out, 'w') as handle:
            handle.write(table)
        print(f'Wrote {args.out}', file=sys.stderr)
    else:
        print(table)
    return 0


if __name__ == '__main__':
    sys.exit(main())
