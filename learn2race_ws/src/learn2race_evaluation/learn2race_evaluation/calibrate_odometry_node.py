#!/usr/bin/env python3
"""Odometry calibration by the UMBmark method.

A straight-line test alone cannot separate the two systematic errors of a
differential drive, because both produce a distance error. The bidirectional
square-path test of Borenstein and Feng does separate them: the robot drives a
square of known side, first clockwise and then anticlockwise, and the way the
final position error differs between the two directions identifies which error
caused it.

    E_d  unequal effective wheel diameters. Makes the robot curve. The curvature
         is in the same rotational sense whichever way the square is driven, so
         it displaces the two end points in opposite directions.
    E_b  effective wheelbase differing from the configured track width. Makes
         every turn the wrong size. The error accumulates the same way in both
         directions, so it displaces both end points the same way.

Clockwise and anticlockwise runs therefore give two equations for two unknowns,
which a single-direction test cannot.

    ros2 run learn2race_evaluation calibrate_odometry --ros-args \\
        -p side_m:=2.0 -p runs:=5

The robot steers by its own odometry, as UMBmark requires: each side ends when
odometry reports side_m travelled and each turn when it reports 90 degrees, so
the measured end error is the odometry error, not acceleration or timing.

UMBmark cannot see a common scale error (both wheels equally too large or
small): the square just grows. mode:=straight measures it against a tape:

    ros2 run learn2race_evaluation calibrate_odometry --ros-args \\
        -p mode:=straight -p side_m:=2.0 -p runs:=3
"""

# References (full entries: docs/references.md at the repository root):
#   [R43] Borenstein & Feng (1996) UMBmark: UMBmark bidirectional square test
#   [R42] Siegwart et al., Autonomous Mobile Robots: differential drive
# More details: README.md of this package

import math
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .calibration_store import CalibrationStore


class CalibrateOdometry(Node):

    def __init__(self):
        super().__init__('calibrate_odometry')

        self.declare_parameter('side_m', 2.0)
        self.declare_parameter('runs', 5)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('turn_rate', 0.6)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('track_width', 0.150)
        self.declare_parameter('config_path', '')
        self.declare_parameter('mode', 'umbmark')        # umbmark | straight
        self.declare_parameter('odom_timeout_s', 0.5)
        # Slowest rotation used near the end of a turn. Turning in place scrubs
        # the tyres; below ~0.2 rad/s Prototype A's motors stall and the turn
        # never finishes.
        self.declare_parameter('min_turn_rate', 0.25)

        self.side = float(self.get_parameter('side_m').value)
        self.runs = int(self.get_parameter('runs').value)
        self.speed = float(self.get_parameter('speed').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)

        self.pose = (0.0, 0.0, 0.0)
        self.last_odom = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.declare_parameter('turns', 2)
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)

        path = self.get_parameter('config_path').value
        self.store = CalibrationStore(path) if path else CalibrationStore()

        threading.Thread(target=self._run, daemon=True).start()

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.last_odom = time.monotonic()

    def _send(self, linear, angular):
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        self.pub.publish(command)

    def _settle(self):
        # Stop, then let the robot come to rest before the next move.
        for _ in range(40):
            self._send(0.0, 0.0)
            time.sleep(0.02)

    def _odom_alive(self):
        timeout = float(self.get_parameter('odom_timeout_s').value)
        return self.last_odom is not None and time.monotonic() - self.last_odom < timeout

    def _drive_distance(self, distance):
        """Drive straight until odometry reports `distance` metres."""
        x0, y0, _ = self.pose
        deadline = time.monotonic() + 3.0 * distance / self.speed + 5.0
        while True:
            done = math.hypot(self.pose[0] - x0, self.pose[1] - y0)
            if not self._odom_alive():
                self._settle()
                raise RuntimeError('no /odom for over odom_timeout_s while driving; aborted')
            if time.monotonic() > deadline:
                self._settle()
                raise RuntimeError(f'drive stalled after {done:.3f} of {distance:.3f} m; aborted')
            remaining = distance - done
            if remaining <= 0.002:
                break
            # Slow down over the last 5 cm so braking does not overshoot.
            self._send(max(0.04, min(self.speed, self.speed * remaining / 0.05)), 0.0)
            time.sleep(0.02)
        self._settle()

    def _turn_angle(self, angle):
        """Rotate until odometry reports `angle` radians, then trim any overshoot.

        UMBmark attributes every heading error to the odometry, so a turn that
        overshoots by braking lag would be misread as a track-width error. After
        the main turn, slow corrective nudges bring the odometry heading within
        0.2 degrees of the target.
        """
        start = self.pose[2]
        self._rotate_by(angle, self.turn_rate)
        for _ in range(3):
            turned = math.atan2(math.sin(self.pose[2] - start), math.cos(self.pose[2] - start))
            error = angle - turned
            if abs(error) <= math.radians(0.3):
                break
            self._rotate_by(error, float(self.get_parameter('min_turn_rate').value))

    def _rotate_by(self, angle, max_rate):
        turned, last = 0.0, self.pose[2]
        deadline = time.monotonic() + 3.0 * abs(angle) / max_rate + 5.0
        while True:
            if not self._odom_alive():
                self._settle()
                raise RuntimeError('no /odom for over odom_timeout_s during a turn; aborted')
            if time.monotonic() > deadline:
                self._settle()
                raise RuntimeError(
                    f'turn stalled after {math.degrees(abs(turned)):.1f} of '
                    f'{math.degrees(abs(angle)):.1f} deg; the motors may stall at low '
                    'rates: raise min_turn_rate')
            yaw = self.pose[2]
            turned += math.atan2(math.sin(yaw - last), math.cos(yaw - last))
            last = yaw
            remaining = abs(angle) - abs(turned)
            if remaining <= math.radians(0.3):
                break
            min_rate = float(self.get_parameter('min_turn_rate').value)
            rate = max(min_rate, min(max_rate, max_rate * remaining / math.radians(10)))
            self._send(0.0, math.copysign(rate, angle))
            time.sleep(0.02)
        self._settle()

    def _square(self, clockwise: bool):
        turn = -math.pi / 2.0 if clockwise else math.pi / 2.0
        for _ in range(4):
            self._drive_distance(self.side)
            self._turn_angle(turn)

    def _wait_for_odom(self):
        end = time.monotonic() + 10.0
        while not self._odom_alive():
            if time.monotonic() > end:
                print('No /odom received. Is the robot bringup running?')
                rclpy.try_shutdown()
                return False
            time.sleep(0.1)
        return True

    def _on_scan(self, msg):
        self.scan = msg

    def _scan_profile(self, samples=5, bins=1440):
        """Median of a few scans resampled to fixed angle bins (NaN = no return)."""
        import numpy as np
        stack = []
        for _ in range(samples):
            self.scan = None
            end = time.monotonic() + 2.0
            while self.scan is None and time.monotonic() < end:
                time.sleep(0.02)
            if self.scan is None:
                raise RuntimeError('no /scan received; is the LiDAR running?')
            m = self.scan
            ranges = np.asarray(m.ranges, dtype=float)
            angles = m.angle_min + np.arange(len(ranges)) * m.angle_increment
            valid = np.isfinite(ranges) & (ranges > max(m.range_min, 0.05)) & (ranges < m.range_max)
            profile = np.full(bins, np.nan)
            idx = np.round((np.mod(angles[valid], 2 * math.pi)) / (2 * math.pi) * bins).astype(int) % bins
            profile[idx] = ranges[valid]
            stack.append(profile)
        return np.nanmedian(np.vstack(stack), axis=0)

    @staticmethod
    def _profile_shift(before, after, max_deg=40.0, center=0.0):
        """Angle s (rad) near `center` with after(theta) ~ before(theta + s)."""
        import numpy as np
        bins = len(before)
        span = int(max_deg / 360.0 * bins)
        c = int(round(center / (2 * math.pi) * bins))
        costs = []
        for k in range(c - span, c + span + 1):
            b = np.roll(before, -k)
            ok = np.isfinite(b) & np.isfinite(after)
            costs.append(np.mean(np.abs(b[ok] - after[ok])) if ok.sum() > bins * 0.2 else np.inf)
        costs = np.asarray(costs)
        i = int(np.argmin(costs))
        frac = 0.0
        if 0 < i < len(costs) - 1 and np.all(np.isfinite(costs[i - 1:i + 2])):
            c0, c1, c2 = costs[i - 1:i + 2]
            denom = c0 - 2 * c1 + c2
            frac = 0.5 * (c0 - c2) / denom if denom > 1e-12 else 0.0
        return (c + i - span + frac) * 2 * math.pi / bins, float(costs[i])

    def _rotation_test(self):
        """Track width from in-place rotations, with the LiDAR as ground truth.

        A pure rotation depends only on the track width and the mean wheel
        radius, not on a left/right wheel difference, so it isolates the track.
        Real rotation = odometry target + the extra angle the room appears
        rotated between LiDAR scans taken before and after.
        """
        turns = int(self.get_parameter('turns').value)
        track = float(self.get_parameter('track_width').value)
        print(f'\nTrack-width test: {turns} full turn(s) in place each way, by odometry,')
        print(f'with the LiDAR measuring the real rotation. Using track_width {track:.5f} m.')
        print('Keep people and moving things out of the LiDAR view; the robot needs')
        print('about 0.5 m clear around it.\n')
        tracks = []
        for sign, name in ((1, 'anticlockwise'), (-1, 'clockwise')):
            input(f'  Press Enter for the {name} test ')
            before = self._scan_profile()
            start = self.pose[2]
            # Probe: one quarter turn gives the per-quarter error, so the final
            # search can be centred where a badly wrong track would land.
            self._turn_angle(sign * math.pi / 2)
            time.sleep(0.5)
            quarter, _ = self._profile_shift(before, self._scan_profile(),
                                             center=sign * math.pi / 2)
            per_quarter_extra = quarter - sign * math.pi / 2
            print(f'    probe: 90 deg by odometry was {math.degrees(quarter):+.1f} deg real')
            # Rotate in quarter turns so the trim logic keeps the odometry exact.
            for _ in range(turns * 4 - 1):
                self._turn_angle(sign * math.pi / 2)
            odom_angle = sign * turns * 2 * math.pi + math.atan2(
                math.sin(self.pose[2] - start), math.cos(self.pose[2] - start))
            time.sleep(0.5)
            after = self._scan_profile()
            extra, cost = self._profile_shift(before, after,
                                              center=per_quarter_extra * turns * 4)
            real = sign * turns * 2 * math.pi + extra
            corrected = track * odom_angle / real
            tracks.append(corrected)
            print(f'    odometry {math.degrees(odom_angle):+8.2f} deg   LiDAR says '
                  f'{math.degrees(real):+8.2f} deg   (match error {cost * 100:.1f} cm)')
            print(f'    -> track width {corrected:.4f} m')
        mean = sum(tracks) / len(tracks)
        print('=' * 66)
        print(f'  track width   {track:.5f} m  ->  {mean:.5f} m   '
              f'(anticlockwise {tracks[0]:.4f}, clockwise {tracks[1]:.4f})')
        print('=' * 66)
        if abs(tracks[0] - tracks[1]) > 0.01 * mean:
            print('  The two directions differ by more than 1%: repeat, and check the')
            print('  robot turns on the spot rather than drifting.')
        self.store.update('odometry_track', {
            'track_width': track, 'corrected_track_width': float(mean),
            'anticlockwise': float(tracks[0]), 'clockwise': float(tracks[1]), 'turns': turns,
        }, tool='calibrate_odometry')
        print(f'\nWritten to {self.store.save()}')
        rclpy.try_shutdown()

    def _straight(self):
        print(f'\nStraight-line scale test: {self.runs} runs of {self.side:.2f} m by odometry.')
        print('Mark the start of the robot (e.g. the front axle) on the floor. After each')
        print('run, measure how far that point actually moved with a tape.\n')
        ratios = []
        for run in range(1, self.runs + 1):
            input(f'  Place the robot on the start mark. Press Enter for run {run}/{self.runs} ')
            x0, y0, _ = self.pose
            self._drive_distance(self.side)
            odom = math.hypot(self.pose[0] - x0, self.pose[1] - y0)
            real = float(input(f'    odometry: {odom:.3f} m. Measured distance (m): '))
            ratios.append(real / odom)
            print(f'    real/odometry = {ratios[-1]:.4f}')
        ratio = sum(ratios) / len(ratios)
        radius = float(self.get_parameter('wheel_radius').value)
        spread = max(ratios) - min(ratios)
        print('=' * 66)
        print(f'  mean real/odometry  {ratio:.4f}  (spread {spread:.4f} over {len(ratios)} runs)')
        print(f'  wheel radius        {radius:.5f} m  ->  {radius * ratio:.5f} m')
        print('=' * 66)
        if spread > 0.02:
            print('  The runs disagree by more than 2%: check for wheel slip and repeat.')
        self.store.update('odometry_scale', {
            'wheel_radius': radius, 'corrected_wheel_radius': float(radius * ratio),
            'real_over_odometry': float(ratio), 'distance_m': self.side, 'runs': self.runs,
        }, tool='calibrate_odometry')
        print(f'\nWritten to {self.store.save()}')
        rclpy.try_shutdown()

    def _run(self):
        if not self._wait_for_odom():
            return
        try:
            mode = self.get_parameter('mode').value
            if mode == 'straight':
                self._straight()
            elif mode == 'rotate':
                self._rotation_test()
            else:
                self._umbmark()
        except RuntimeError as exc:
            print(f'\n  {exc}')
            rclpy.try_shutdown()

    def _umbmark(self):
        print('\nUMBmark odometry calibration.')
        print(f'Using wheel_radius {float(self.get_parameter("wheel_radius").value):.5f} m and '
              f'track_width {float(self.get_parameter("track_width").value):.5f} m: '
              'these must be the values the firmware uses.')
        print(f'The robot drives a {self.side:.1f} m square, {self.runs} times '
              f'clockwise then {self.runs} times anticlockwise.')
        print('Mark the exact start position and heading on the floor. After '
              'each run, measure where the robot actually stopped relative to '
              'that mark, in the robot\'s own start frame.\n')

        results = {}
        for clockwise in (True, False):
            name = 'clockwise' if clockwise else 'anticlockwise'
            errors = []
            for run in range(1, self.runs + 1):
                input(f'  Place the robot on the mark. Press Enter for '
                      f'{name} run {run}/{self.runs} ')
                self._square(clockwise)
                print(f'    odometry believes it is at '
                      f'x={self.pose[0]:+.3f} y={self.pose[1]:+.3f} '
                      f'yaw={math.degrees(self.pose[2]):+.1f} deg')
                x = float(input('    measured x error (m, forward positive): '))
                y = float(input('    measured y error (m, left positive):    '))
                errors.append((x, y))

            cx = sum(e[0] for e in errors) / len(errors)
            cy = sum(e[1] for e in errors) / len(errors)
            results[name] = (cx, cy)
            print(f'  centre of gravity of {name} errors: '
                  f'x={cx:+.4f} y={cy:+.4f}\n')

        self._compute(results)

    def _compute(self, results):
        cw_x, cw_y = results['clockwise']
        ccw_x, ccw_y = results['anticlockwise']

        # Borenstein and Feng: the half-difference and half-sum of the two
        # centres of gravity separate the two systematic errors.
        alpha = ((cw_x + ccw_x) / (-4.0 * self.side)) * (180.0 / math.pi)
        beta = ((cw_x - ccw_x) / (-4.0 * self.side)) * (180.0 / math.pi)

        radius = float(self.get_parameter('wheel_radius').value)
        track = float(self.get_parameter('track_width').value)

        # E_b scales the wheelbase. E_d (right/left effective diameter) follows
        # Borenstein & Feng: the curvature radius R = (L/2)/sin(beta/2) of the
        # path, then E_d = (R + b/2)/(R - b/2) with b the wheelbase. (An
        # earlier version used L/2 in place of R and b, which overstated the
        # wheel difference about fivefold.)
        e_b = 90.0 / (90.0 - alpha) if abs(90.0 - alpha) > 1e-6 else 1.0
        if abs(beta) > 1e-9:
            curvature_radius = (self.side / 2.0) / math.sin(math.radians(beta) / 2.0)
            radius_ratio = ((curvature_radius + track / 2.0) /
                            (curvature_radius - track / 2.0))
        else:
            radius_ratio = 1.0

        corrected_track = track * e_b
        # Split E_d between the wheels keeping their mean radius unchanged.
        left_radius = radius * 2.0 / (1.0 + radius_ratio)
        right_radius = radius * 2.0 / (1.0 + 1.0 / radius_ratio)

        print('=' * 66)
        print(f'  alpha (wheelbase error)        {alpha:+8.4f} deg')
        print(f'  beta  (wheel diameter error)   {beta:+8.4f} deg')
        print()
        print(f'  track width   {track:.5f} m  ->  {corrected_track:.5f} m')
        print(f'  wheel diameter ratio (right/left)  {radius_ratio:.5f}')
        print(f'  wheel radius  left {left_radius:.5f} m   right {right_radius:.5f} m')
        print()
        if abs(alpha) > abs(beta) * 2.0:
            print('  The wheelbase term dominates: the configured track width '
                  'is the value to correct.')
        elif abs(beta) > abs(alpha) * 2.0:
            print('  The wheel diameter term dominates: the wheels differ, '
                  'from wear or from tyre seating. Correct per wheel rather '
                  'than adjusting the track width.')
        else:
            print('  Both terms are comparable; apply both corrections.')
        print('=' * 66)

        self.store.update('odometry', {
            'wheel_radius': radius,
            'track_width': float(corrected_track),
            'wheel_radius_ratio_right_left': float(radius_ratio),
            'left_wheel_radius': float(left_radius),
            'right_wheel_radius': float(right_radius),
            'umbmark_alpha_deg': float(alpha),
            'umbmark_beta_deg': float(beta),
            'square_side_m': self.side,
            'runs_per_direction': self.runs,
        }, tool='calibrate_odometry')
        print(f'\nWritten to {self.store.save()}')
        print('Note: this corrects systematic error only. Non-systematic error '
              'from wheel slip is not removed by calibration.')
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
