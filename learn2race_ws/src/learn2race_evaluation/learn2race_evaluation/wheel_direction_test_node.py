#!/usr/bin/env python3
"""Wheel direction test: find the four sign flags from observation.

Each driven wheel has two independent directions to get right, because the motor
leads and the encoder channels can each be wired either way round:

* the MOTOR sign decides which way the wheel turns for a forward command;
* the ENCODER sign decides whether that motion is measured as forward.

The symptoms differ, which is what makes them separable:

    motor ok,    encoder ok     wheel turns forward at the commanded speed
    motor WRONG, encoder ok     wheel turns BACKWARDS and races to full speed
    motor ok,    encoder WRONG  wheel turns forward but races to full speed
    both WRONG                  wheel turns backwards at a steady speed

A wheel that races is the speed controller fighting a measurement of the wrong
sign. /odom distinguishes the encoder cases: with one encoder reversed, driving
straight reads as rotating in place, and the direction of that apparent
rotation says which side is wrong.

Run with the robot LIFTED so the wheels spin freely: a sign error makes a wheel
run away, and on the floor that is a robot running away.

    ros2 launch learn2race_bringup prototype_a.launch.py competition:=false teleop:=true
    ros2 run learn2race_evaluation wheel_direction_test

Without the joystick packages installed, start the launch without teleop and
send the test commands on the competition channel instead, which the command
multiplexer always passes through:

    ros2 launch learn2race_bringup prototype_a.launch.py competition:=false
    ros2 run learn2race_evaluation wheel_direction_test --ros-args \\
        -p topic:=competition/cmd_vel
"""

# References (full entries: docs/references.md at the repository root):
#   [R63] ESP32 TRM / Arduino-ESP32 / PlatformIO: encoder counting
#   [R42] Siegwart et al., Autonomous Mobile Robots: differential drive
# More details: README.md of this package

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class WheelDirectionTest(Node):

    def __init__(self):
        super().__init__('wheel_direction_test')
        self.declare_parameter('topic', 'teleop/cmd_vel')
        self.declare_parameter('speed', 0.08)
        self.declare_parameter('seconds', 2.0)
        self.declare_parameter('runaway_mps', 0.4)

        self.linear = []
        self.angular = []
        self.recording = False
        self.pub = self.create_publisher(Twist, self.get_parameter('topic').value, 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        threading.Thread(target=self._run, daemon=True).start()

    def _on_odom(self, msg: Odometry):
        if self.recording:
            self.linear.append(msg.twist.twist.linear.x)
            self.angular.append(msg.twist.twist.angular.z)

    def _drive(self, linear, angular, seconds):
        self.linear, self.angular = [], []
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        limit = float(self.get_parameter('runaway_mps').value)
        self.recording = True
        end = time.monotonic() + seconds
        runaway = False
        while time.monotonic() < end:
            self.pub.publish(command)
            if self.linear and abs(self.linear[-1]) > limit:
                runaway = True
                break
            time.sleep(0.04)
        self.recording = False
        for _ in range(10):
            self.pub.publish(Twist())
            time.sleep(0.04)
        return runaway

    @staticmethod
    def _mean(values):
        tail = values[len(values) // 3:] or values
        return sum(tail) / len(tail) if tail else 0.0

    def _ask(self, prompt):
        while True:
            answer = input(prompt).strip().lower()
            if answer in ('f', 'b', 'n'):
                return answer
            print('    answer f (forward), b (backward) or n (not moving)')

    def _run(self):
        speed = float(self.get_parameter('speed').value)
        seconds = float(self.get_parameter('seconds').value)

        print('\nWheel direction test.')
        print('LIFT THE ROBOT so both driven wheels spin freely, then press Enter.')
        print('Watch each wheel during the run: forward means the direction that')
        print('would move the robot forward if it were on the floor.\n')
        input('  Ready? ')

        runaway = self._drive(speed, 0.0, seconds)
        lin, ang = self._mean(self.linear), self._mean(self.angular)

        if not self.linear:
            print('\n  No /odom received. In order, check:')
            print('    1. Did the launch file actually start? A missing package')
            print('       aborts the whole launch, so nothing runs at all.')
            print('    2. ros2 topic hz /odom  should show about 20 Hz.')
            print('    3. This test publishes to '
                  f'{self.get_parameter("topic").value}. With competition mode on,')
            print('       teleop/cmd_vel is blocked by the multiplexer. Either')
            print('       start the launch with teleop:=true, or re-run this test')
            print('       with -p topic:=competition/cmd_vel, which is always')
            print('       passed through.')
            rclpy.try_shutdown()
            return

        print(f'\n  /odom during a straight forward command: '
              f'linear {lin:+.3f} m/s, angular {ang:+.3f} rad/s'
              + ('  (stopped early: a wheel ran away)' if runaway else ''))

        left = self._ask('  Which way did the LEFT wheel turn?  f / b / n: ')
        right = self._ask('  Which way did the RIGHT wheel turn? f / b / n: ')

        flips = []
        for side, seen in (('LEFT', left), ('RIGHT', right)):
            if seen == 'b':
                flips.append(f'{side}_MOTOR_SIGN')
            if seen == 'n':
                print(f'\n  The {side.lower()} wheel did not move. Check its motor '
                      f'wiring and that MIN_PWM_DUTY is above its stall threshold.')

        # With both wheels physically forward, odometry exposes encoder errors.
        if left == 'f' and right == 'f':
            if lin < 0 and abs(ang) < abs(lin) * 2:
                flips += ['LEFT_ENCODER_SIGN', 'RIGHT_ENCODER_SIGN']
            elif abs(ang) > 0.3 and abs(lin) < speed * 0.5:
                # One encoder reversed: straight motion reads as rotation. A
                # reversed right encoder reads the right wheel as going
                # backwards, which looks like turning right (negative yaw).
                flips.append('RIGHT_ENCODER_SIGN' if ang < 0 else 'LEFT_ENCODER_SIGN')
        elif runaway and not flips:
            print('\n  A wheel ran away with no motor sign error reported: an '
                  'encoder sign is reversed. Flip the motor sign first if any '
                  'wheel turned backwards, re-flash, and repeat the test.')

        print('\n' + '=' * 62)
        if flips:
            print('  In platformio.ini, change the sign of:')
            for flag in flips:
                print(f'    {flag}')
            print('  Re-flash, then run this test again until nothing is listed.')
        elif left == 'f' and right == 'f':
            print('  Both wheels turn forward and odometry agrees. The signs are')
            print('  correct. Next: check the turn direction below.')
        print('=' * 62)

        if left == 'f' and right == 'f' and not flips:
            input('\n  Now a LEFT turn in place. Press Enter. ')
            self._drive(0.0, 1.0, seconds)
            yaw = self._mean(self.angular)
            print(f'  /odom angular {yaw:+.3f} rad/s during a left turn command')
            if yaw > 0.2:
                print('  Correct: a left turn reads as positive yaw, as ROS requires.')
            elif yaw < -0.2:
                print('  Reversed: swap LEFT and RIGHT in RobotApp (the wheels are '
                      'assigned to the wrong sides).')
            else:
                print('  Little rotation measured. Check the track width setting.')

        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WheelDirectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
