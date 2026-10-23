#!/usr/bin/env python3
"""First-line hardware check: one command from power-on to a PASS/FAIL list (NFR5).

Run through hardware_check.launch.py, which starts the robot's bringup outside
competition mode and then this node. It works through the robot in order, from
the bus up, so the first failure points at the layer that is broken:

  1. robot.yaml       the robot description is present and readable
  2. I2C devices      each fixed device on the Pi's bus answers at its address
                      (absent: wiring or power; present but its topic silent:
                      driver or configuration)
  3. ESP32 link       /odom arrives, so the micro-ROS agent, the serial link and
                      the firmware are up
  4. topics           every sensor topic in robot.yaml publishes at its rate
  5. status LED       red, green, blue, white in turn; watch the LED
  6. motors/encoders  only with wheels_lifted:=true: a slow forward command and
                      a left turn in place, compared with /odom (sign, speed,
                      and whether a straight command reads as a turn)

    ~/robot.sh check                         # everything except the motors
    ~/robot.sh check wheels_lifted:=true     # robot on a stand, wheels free

The report is printed and saved to ~/run_logs/hardware_check-<time>.txt. For a
wheel that fails, wheel_direction_test finds which sign flag is wrong.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: NFR5 documented procedures
#   [R42] Siegwart et al., Autonomous Mobile Robots: differential drive
# More details: README.md of this package

import errno
import importlib
import math
import os
import threading
import time

import rclpy
import rclpy.executors
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, ColorRGBA, String

NACK = {errno.ENXIO, errno.EREMOTEIO, errno.EIO, errno.ETIMEDOUT}


def find_robot_yaml(explicit):
    """robot.yaml: the parameter, $L2R_ROBOT_YAML, or beside ~/learn2race_ws."""
    candidates = [explicit, os.environ.get('L2R_ROBOT_YAML', '')]
    workspace = os.path.realpath(os.path.expanduser('~/learn2race_ws'))
    candidates.append(os.path.join(os.path.dirname(workspace), 'robot.yaml'))
    for path in candidates:
        if path and os.path.isfile(os.path.expanduser(path)):
            return os.path.expanduser(path)
    return None


def message_class(type_name):
    """'nav_msgs/msg/Odometry' -> the Python class."""
    package, _, name = type_name.split('/')
    return getattr(importlib.import_module(f'{package}.msg'), name)


def probe_i2c(bus, address):
    """True if a device answers, as i2cdetect does (quick write, or read byte)."""
    try:
        if 0x30 <= address <= 0x37 or 0x50 <= address <= 0x5F:
            bus.read_byte(address)
        else:
            try:
                bus.write_quick(address)
            except OSError as error:
                if error.errno in NACK:
                    raise
                bus.read_byte(address)    # adapter without SMBus quick
        return True, ''
    except OSError as error:
        if error.errno in NACK:
            return False, ''
        return False, f'bus error: {error}'


class HardwareCheck(Node):

    def __init__(self):
        super().__init__('hardware_check')
        self.declare_parameter('robot_yaml', '')
        self.declare_parameter('wheels_lifted', False)
        self.declare_parameter('speed', 0.08)            # m/s, forward test
        self.declare_parameter('turn_rate', 0.6)         # rad/s, turn test
        self.declare_parameter('drive_seconds', 2.0)
        self.declare_parameter('runaway_mps', 0.4)
        self.declare_parameter('wait_seconds', 30.0)     # for /odom and the safety monitor
        self.declare_parameter('topic_window', 8.0)
        self.declare_parameter('drive_topic', 'competition/cmd_vel')  # always passed by the mux
        self.declare_parameter('report_dir', '~/run_logs')

        self.results = []          # (status, item, detail)
        self.lock = threading.Lock()
        self.odom = []             # (t, v, w)
        self.counts = {}
        self.safety = {'ready': None, 'fault': None, 'reason': ''}

        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, 'safety/ready', lambda m: self._safety('ready', m.data), 10)
        self.create_subscription(Bool, 'safety/fault', lambda m: self._safety('fault', m.data), 10)
        self.create_subscription(String, 'safety/fault_reason',
                                 lambda m: self._safety('reason', m.data), 10)
        self.led_pub = self.create_publisher(ColorRGBA, 'leds/set_color', 10)
        self.drive_pub = self.create_publisher(
            Twist, self.get_parameter('drive_topic').value, 10)
        threading.Thread(target=self._run, daemon=True).start()

    # --- callbacks ---------------------------------------------------------
    def _on_odom(self, msg):
        with self.lock:
            self.odom.append((time.monotonic(), msg.twist.twist.linear.x,
                              msg.twist.twist.angular.z))
            del self.odom[:-400]

    def _safety(self, key, value):
        self.safety[key] = value

    def _count(self, topic, _msg):
        with self.lock:
            self.counts[topic] = self.counts.get(topic, 0) + 1

    # --- helpers -----------------------------------------------------------
    def p(self, name):
        return self.get_parameter(name).value

    def record(self, status, item, detail=''):
        self.results.append((status, item, detail))
        print(f'  {status:<5} {item:<34} {detail}', flush=True)

    def odom_since(self, start):
        with self.lock:
            return [(v, w) for t, v, w in self.odom if t >= start]

    def drive(self, linear, angular, seconds):
        """Command for `seconds` at 20 Hz; return (v, w) means over the last 70 %."""
        command = Twist()
        command.linear.x, command.angular.z = float(linear), float(angular)
        start = time.monotonic()
        settle = start + 0.3 * seconds
        runaway = float(self.p('runaway_mps'))
        while time.monotonic() - start < seconds:
            self.drive_pub.publish(command)
            recent = self.odom_since(time.monotonic() - 0.3)
            # A sign error makes a wheel race to full speed: as forward speed,
            # or as rotation when only one side runs away.
            if recent and (abs(recent[-1][0]) > runaway or abs(recent[-1][1]) > 4.0):
                self.stop()
                return None
            time.sleep(0.05)
        samples = self.odom_since(settle)
        self.stop()
        if not samples:
            return float('nan'), float('nan')
        return (sum(v for v, _ in samples) / len(samples),
                sum(w for _, w in samples) / len(samples))

    def stop(self):
        zero = Twist()
        for _ in range(20):
            self.drive_pub.publish(zero)
            time.sleep(0.05)

    # --- the sequence ------------------------------------------------------
    def _run(self):
        time.sleep(1.0)
        print('\nLearn2Race hardware check\n' + '-' * 62, flush=True)
        try:
            robot = self.check_description()
            if robot is not None:
                self.check_i2c(robot)
                if self.check_link():
                    self.check_topics(robot)
                    self.check_led()
                    self.check_motors()
        except Exception as error:     # report, never leave the wheels turning
            self.stop()
            self.record('FAIL', 'hardware check aborted', repr(error))
        self.finish()

    def check_description(self):
        path = find_robot_yaml(self.p('robot_yaml'))
        if path is None:
            self.record('FAIL', 'robot.yaml', 'not found beside ~/learn2race_ws; '
                        'set robot_yaml:=PATH')
            return None
        with open(path, encoding='utf-8') as handle:
            robot = yaml.safe_load(handle)
        name = robot.get('robot', {}).get('name', '?')
        self.record('PASS', 'robot.yaml', f'{name} ({path})')
        self.robot_name = name
        return robot

    def check_i2c(self, robot):
        section = robot.get('i2c') or {}
        devices = section.get('devices') or []
        if not devices:
            return
        try:
            from smbus2 import SMBus
            bus = SMBus(int(section.get('bus', 1)))
        except (ImportError, OSError) as error:
            self.record('FAIL', f"I2C bus {section.get('bus', 1)}", f'cannot open: {error}')
            return
        with bus:
            for device in devices:
                address = int(device['address'])
                present, problem = probe_i2c(bus, address)
                item = f"I2C 0x{address:02x} {device['name']}"
                if present:
                    self.record('PASS', item, 'answers')
                elif problem:
                    self.record('FAIL', item, problem)
                elif device.get('required', True):
                    self.record('FAIL', item, 'no answer: check its wiring and power')
                else:
                    self.record('WARN', item, 'no answer (optional device)')

    def check_link(self):
        deadline = time.monotonic() + float(self.p('wait_seconds'))
        while time.monotonic() < deadline:
            if self.odom_since(time.monotonic() - 1.0):
                self.record('PASS', 'ESP32 link (/odom)', 'agent, serial link and firmware up')
                return True
            time.sleep(0.2)
        self.record('FAIL', 'ESP32 link (/odom)',
                    f"no /odom in {self.p('wait_seconds'):.0f} s: ESP32 power, USB/UART cable, "
                    'serial device or baud rate (robot.yaml firmware section)')
        return False

    def check_topics(self, robot):
        topics = robot.get('topics') or []
        subscriptions = []
        for entry in topics:
            try:
                cls = message_class(entry['type'])
            except (ImportError, AttributeError, ValueError) as error:
                self.record('FAIL', entry['name'], f'unknown type {entry["type"]}: {error}')
                continue
            subscriptions.append(self.create_subscription(
                cls, entry['name'], lambda m, t=entry['name']: self._count(t, m),
                qos_profile_sensor_data))
        window = float(self.p('topic_window'))
        with self.lock:
            self.counts = {}
        time.sleep(window)
        with self.lock:
            counts = dict(self.counts)
        for entry in topics:
            rate = counts.get(entry['name'], 0) / window
            minimum = float(entry['min_hz'])
            if rate >= minimum:
                self.record('PASS', f"topic {entry['name']}", f'{rate:.1f} Hz (min {minimum:g})')
            elif rate > 0:
                self.record('FAIL', f"topic {entry['name']}",
                            f'{rate:.1f} Hz, below {minimum:g}: bus contention or wrong device setting')
            else:
                self.record('FAIL', f"topic {entry['name']}",
                            'silent: driver not started, or the device is missing')
        for subscription in subscriptions:
            self.destroy_subscription(subscription)

    def check_led(self):
        if self.count_subscribers('leds/set_color') == 0:
            self.record('FAIL', 'status LED', 'nothing subscribes to leds/set_color')
            return
        print('        Watch the status LED: red, green, blue, white ...', flush=True)
        for name, rgb in (('red', (1.0, 0.0, 0.0)), ('green', (0.0, 1.0, 0.0)),
                          ('blue', (0.0, 0.0, 1.0)), ('white', (1.0, 1.0, 1.0))):
            colour = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                self.led_pub.publish(colour)
                time.sleep(0.1)
        self.led_pub.publish(ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0))
        self.record('LOOK', 'status LED', 'showed red, green, blue, white? (check by eye)')

    def check_motors(self):
        if not self.p('wheels_lifted'):
            self.record('SKIP', 'motors and encoders', 'lift the robot and add wheels_lifted:=true')
            return
        # The multiplexer passes commands only once the safety monitor reports
        # ready (Prototype A) and no fault is latched.
        wait_ready = self.count_publishers('safety/ready') > 0
        deadline = time.monotonic() + float(self.p('wait_seconds'))
        while time.monotonic() < deadline:
            ready = self.safety['ready'] is True or not wait_ready
            if ready and self.safety['fault'] is not True:
                break
            time.sleep(0.2)
        else:
            reason = self.safety['reason'] or 'not ready'
            self.record('FAIL', 'motors and encoders', f'safety monitor blocks driving: {reason}')
            return

        speed, turn = float(self.p('speed')), float(self.p('turn_rate'))
        seconds = float(self.p('drive_seconds'))
        print(f'        Driving: forward {speed} m/s, then a left turn {turn} rad/s ...', flush=True)
        forward = self.drive(speed, 0.0, seconds)
        time.sleep(0.5)
        left = self.drive(0.0, turn, seconds) if forward is not None else None
        if forward is None or left is None:
            self.record('FAIL', 'motors and encoders',
                        'a wheel ran away (encoder sign or motor sign wrong): '
                        'run wheel_direction_test')
            return

        v, w = forward
        if math.isnan(v):
            self.record('FAIL', 'drive forward', 'no /odom during the test')
        elif abs(v) < 0.2 * speed and abs(w) > 0.25 * turn:
            self.record('FAIL', 'drive forward', f'/odom {v:+.3f} m/s, {w:+.2f} rad/s: a straight '
                        'command reads as a turn, so one encoder is reversed: run wheel_direction_test')
        elif abs(v) < 0.2 * speed:
            self.record('FAIL', 'drive forward', f'/odom {v:+.3f} m/s: wheels not turning or '
                        'encoders not counting (motor power, H-bridge, encoder cable)')
        elif v < 0:
            self.record('FAIL', 'drive forward', f'/odom {v:+.3f} m/s: reads backwards '
                        '(signs): run wheel_direction_test')
        elif abs(w) > 0.25 * turn:
            self.record('FAIL', 'drive forward', f'/odom {v:+.3f} m/s but turning {w:+.2f} rad/s: '
                        'one side not driving or one encoder reversed')
        elif not 0.5 * speed <= v <= 1.5 * speed:
            self.record('WARN', 'drive forward', f'/odom {v:+.3f} m/s for {speed} commanded: '
                        'check wheel radius and gear ratio')
        else:
            self.record('PASS', 'drive forward', f'/odom {v:+.3f} m/s for {speed} commanded')

        v, w = left
        if math.isnan(w):
            self.record('FAIL', 'turn left in place', 'no /odom during the test')
        elif w < -0.2 * turn:
            self.record('FAIL', 'turn left in place', f'/odom {w:+.2f} rad/s: reversed '
                        '(left and right wheels swapped)')
        elif abs(w) <= 0.2 * turn and abs(v) > 0.2 * speed:
            self.record('FAIL', 'turn left in place', f'/odom {v:+.3f} m/s, {w:+.2f} rad/s: a turn '
                        'reads as driving straight, so one encoder is reversed')
        elif abs(w) <= 0.2 * turn:
            self.record('FAIL', 'turn left in place', f'/odom {w:+.2f} rad/s: no rotation measured')
        elif not 0.5 * turn <= w <= 1.5 * turn:
            self.record('WARN', 'turn left in place', f'/odom {w:+.2f} rad/s for {turn} commanded: '
                        'check the track width')
        else:
            self.record('PASS', 'turn left in place', f'/odom {w:+.2f} rad/s for {turn} commanded')

    def finish(self):
        failures = sum(1 for status, _, _ in self.results if status == 'FAIL')
        warnings = sum(1 for status, _, _ in self.results if status == 'WARN')
        summary = (f'{len(self.results)} checks: {failures} failed, {warnings} warnings'
                   + ('' if failures else ' - hardware OK'))
        print('-' * 62 + f'\n  {summary}\n', flush=True)
        folder = os.path.expanduser(self.p('report_dir'))
        try:
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, time.strftime('hardware_check-%Y%m%d-%H%M%S.txt'))
            with open(path, 'w', encoding='utf-8') as handle:
                handle.write(f'Learn2Race hardware check, {time.strftime("%Y-%m-%d %H:%M:%S")}, '
                             f'{getattr(self, "robot_name", "?")}\n')
                for status, item, detail in self.results:
                    handle.write(f'{status:<5} {item:<34} {detail}\n')
                handle.write(summary + '\n')
            print(f'  Report: {path}', flush=True)
        except OSError as error:
            print(f'  Report not saved: {error}', flush=True)
        # The report is the result; exit 0 so the launch ends without an error.
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
