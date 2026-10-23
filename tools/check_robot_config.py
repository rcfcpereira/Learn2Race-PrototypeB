#!/usr/bin/env python3
"""Check that robot.yaml agrees with the files the robot is built from (NFR3).

robot.yaml records the robot's identity and calibration constants in one
place, but the ESP32 is compiled from firmware/platformio.ini (with defaults in
RobotConfig.h) and the ROS nodes read the bringup config. A constant changed in
one file and not the others gives odometry that silently disagrees with the
record. This script compares them and exits non-zero on any difference.

    python3 tools/check_robot_config.py              # from the repository root
    python3 tools/check_robot_config.py --root PATH  # another checkout

tools/update_robot.sh runs it before every rebuild and refuses to build a
revision that fails.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: NFR3 versioned system state
# More details: INSTALL.md, section "Automatic updates"

import argparse
import configparser
import math
import os
import re
import sys

import yaml

# robot.yaml key -> firmware macro
DRIVETRAIN_MACROS = {
    'gear_ratio': 'GEAR_RATIO',
    'encoder_ppr': 'ENCODER_PPR',
    'wheel_radius_m': 'WHEEL_RADIUS_M',
    'left_wheel_radius_m': 'LEFT_WHEEL_RADIUS_M',
    'right_wheel_radius_m': 'RIGHT_WHEEL_RADIUS_M',
    'track_width_m': 'TRACK_WIDTH_M',
    'max_pwm_duty': 'MAX_PWM_DUTY',
    'motor_count': 'MOTOR_COUNT',
}
TIMING_MACROS = {
    'control_period_ms': 'CONTROL_PERIOD_MS',
    'odom_publish_period_ms': 'ODOM_PUBLISH_PERIOD_MS',
    'cmd_timeout_ms': 'CMD_TIMEOUT_MS',
}
# robot.yaml drivetrain key -> key in the bringup config's odometry block
ROS_ODOMETRY_KEYS = {
    'wheel_radius_m': 'wheel_radius',
    'track_width_m': 'track_width',
    'encoder_ppr': 'encoder_ppr',
    'gear_ratio': 'gear_ratio',
}


def read_platformio(path, env):
    """Return (macros from -D flags, lib_deps text) for one environment."""
    parser = configparser.ConfigParser(interpolation=None, strict=False,
                                       inline_comment_prefixes=(';',))
    parser.optionxform = str
    with open(path, encoding='utf-8') as handle:
        parser.read_file(handle)

    def option(section, key, seen=()):
        name = f'env:{section}'
        if name in seen:
            raise ValueError(f'circular extends at {name}')
        if not parser.has_section(name):
            raise ValueError(f'{path}: no [{name}] section')
        if parser.has_option(name, key):
            value = parser.get(name, key)
        elif parser.has_option(name, 'extends'):
            value = option(parser.get(name, 'extends').strip().split(':')[-1], key,
                           seen + (name,))
        else:
            return ''
        # ${env:other.key} references
        return re.sub(r'\$\{env:([\w-]+)\.([\w_]+)\}',
                      lambda m: option(m.group(1), m.group(2), seen + (name,)), value)

    flags = option(env, 'build_flags')
    macros = {}
    for match in re.finditer(r'-D\s*(\w+)(?:=(\S+))?', flags):
        macros[match.group(1)] = match.group(2) if match.group(2) is not None else '1'
    return macros, option(env, 'lib_deps'), option(env, 'monitor_speed')


def read_header_defaults(path):
    """#define NAME value lines of RobotConfig.h (the defaults the flags override)."""
    defaults = {}
    if not os.path.exists(path):
        return defaults
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            match = re.match(r'\s*#define\s+(\w+)\s+(.+?)\s*(//.*)?$', line)
            if match:
                defaults.setdefault(match.group(1), match.group(2))
    return defaults


def evaluate(name, macros, depth=0):
    """Numeric value of a macro, following references like (MOTOR_COUNT * 20)."""
    if depth > 10 or name not in macros:
        return None
    text = macros[name].rstrip('fFuUlL') if re.fullmatch(r'[\d.]+[fFuUlL]*', macros[name]) \
        else macros[name]

    def substitute(match):
        value = evaluate(match.group(0), macros, depth + 1)
        if value is None:
            raise ValueError(f'unknown macro {match.group(0)} in {name}')
        return repr(value)

    expression = re.sub(r'[A-Za-z_]\w*', substitute, text)
    expression = re.sub(r'(\d)[fFuUlL]+\b', r'\1', expression)
    if not re.fullmatch(r'[\d.\s()+\-*/]+', expression):
        raise ValueError(f'cannot evaluate {name} = {macros[name]}')
    return eval(expression, {'__builtins__': {}})  # digits and operators only


def close(a, b):
    return math.isclose(float(a), float(b), rel_tol=1e-6, abs_tol=1e-9)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', default=os.path.join(os.path.dirname(
        os.path.abspath(__file__)), os.pardir))
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    with open(os.path.join(root, 'robot.yaml'), encoding='utf-8') as handle:
        robot = yaml.safe_load(handle)

    problems, checked = [], 0

    def compare(label, recorded, actual, source):
        nonlocal checked
        checked += 1
        if actual is None:
            problems.append(f'{label}: robot.yaml has {recorded}, {source} has no value')
        elif not close(recorded, actual):
            problems.append(f'{label}: robot.yaml has {recorded}, {source} has {actual}')

    # --- firmware ------------------------------------------------------------
    fw = robot['firmware']
    ini = os.path.join(root, fw['platformio_ini'])
    flags, lib_deps, monitor_speed = read_platformio(ini, fw['environment'])
    header = read_header_defaults(os.path.join(
        root, 'firmware', 'lib', 'RobotConfig', 'src', 'RobotConfig.h'))
    macros = {**header, **flags}          # -D flags win: the header uses #ifndef
    source = f"{fw['platformio_ini']} [env:{fw['environment']}] / RobotConfig.h"

    compare('firmware.serial_baud', fw['serial_baud'], evaluate('ROS_SERIAL_BAUD', macros), source)
    if monitor_speed:
        compare('firmware.serial_baud (monitor_speed)', fw['serial_baud'], monitor_speed,
                fw['platformio_ini'])
    checked += 1
    if fw['micro_ros_arduino'] not in lib_deps:
        problems.append(f"firmware.micro_ros_arduino: robot.yaml has {fw['micro_ros_arduino']}, "
                        f'lib_deps has {lib_deps.split() or "nothing"}')

    for section, table in (('drivetrain', DRIVETRAIN_MACROS), ('timing', TIMING_MACROS)):
        for key, macro in table.items():
            if key in robot.get(section, {}):
                compare(f'{section}.{key}', robot[section][key], evaluate(macro, macros), source)

    drive = robot.get('drivetrain', {})
    if 'counts_per_wheel_rev' in drive:
        compare('drivetrain.counts_per_wheel_rev', drive['counts_per_wheel_rev'],
                evaluate('GEAR_RATIO', macros) * evaluate('ENCODER_PPR', macros) * 4,
                'GEAR_RATIO x ENCODER_PPR x 4')

    # --- ROS configuration ---------------------------------------------------
    config_path = robot['robot']['bringup_config']
    with open(os.path.join(root, config_path), encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    odometry = (config.get('odometry') or {}).get('ros__parameters', {})
    for key, ros_key in ROS_ODOMETRY_KEYS.items():
        if key in drive and ros_key in odometry:
            compare(f'drivetrain.{key}', drive[key], odometry[ros_key],
                    f'{config_path} odometry.{ros_key}')

    launch_path = os.path.join(root, 'learn2race_ws', 'src', 'learn2race_bringup', 'launch',
                               robot['robot']['bringup_launch'])
    with open(launch_path, encoding='utf-8') as handle:
        launch = handle.read()
    for argument, recorded in (('agent_baud', fw['serial_baud']),
                               ('serial_port', fw['serial_device'])):
        match = re.search(rf"DeclareLaunchArgument\(\s*['\"]{argument}['\"],\s*"
                          rf"default_value=['\"]([^'\"]+)['\"]", launch)
        checked += 1
        if not match:
            problems.append(f'{argument}: no default in {robot["robot"]["bringup_launch"]}')
        elif str(match.group(1)) != str(recorded):
            problems.append(f'firmware.{"serial_baud" if argument == "agent_baud" else "serial_device"}: '
                            f'robot.yaml has {recorded}, {robot["robot"]["bringup_launch"]} '
                            f'{argument} defaults to {match.group(1)}')

    name = robot['robot'].get('name')
    params = (config.get('/**') or {}).get('ros__parameters', {})
    if name and 'robot_name' in params:
        checked += 1
        if params['robot_name'] != name:
            problems.append(f"robot.name: robot.yaml has {name}, {config_path} has "
                            f"{params['robot_name']}")

    label = f"Prototype {robot['robot']['prototype'].upper()}"
    if problems:
        print(f'robot.yaml check FAILED for {label} ({len(problems)} of {checked} values differ):')
        for problem in problems:
            print(f'  - {problem}')
        return 1
    print(f'robot.yaml check passed for {label}: {checked} values agree with the firmware '
          f'and ROS configuration.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
