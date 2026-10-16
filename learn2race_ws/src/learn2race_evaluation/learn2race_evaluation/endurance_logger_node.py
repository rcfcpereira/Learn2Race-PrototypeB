#!/usr/bin/env python3
"""Unattended endurance logger (FR11).

The Dragster closed-park rule surrenders the robot at least five minutes before
a round with no further access, so the platform must survive a whole round
untouched. The load is dominated by standing draw rather than by traction,
because the robot spends most of a round powered but stationary.

Logs pack voltage and derived state of charge to CSV at a low rate, and raises
a warning as the pack approaches the configured floor so that a trial can be
stopped before the cells are damaged.

    ros2 run learn2race_evaluation endurance_logger --ros-args \\
        -p cells:=3 -p output:=endurance_a.csv
"""

# References (full entries: docs/references.md at the repository root):
#   [R2]  Dragster rules: closed-park rule
#   [R69] INA219 / INA226 datasheets, INA219_WE: pack current/voltage
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR11
# More details: README.md of this package

import csv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class EnduranceLogger(Node):

    def __init__(self):
        super().__init__('endurance_logger')

        self.declare_parameter('cells', 3)
        self.declare_parameter('cell_floor_volts', 3.5)
        self.declare_parameter('cell_full_volts', 4.2)
        self.declare_parameter('period_seconds', 10.0)
        self.declare_parameter('output', 'endurance.csv')

        self.cells = int(self.get_parameter('cells').value)
        self.floor = self.cells * float(self.get_parameter('cell_floor_volts').value)
        self.full = self.cells * float(self.get_parameter('cell_full_volts').value)

        self.voltage = None
        self.current = None
        self.started = self._now()
        self.warned = False

        path = self.get_parameter('output').value
        self.csv_file = open(path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            ['elapsed_s', 'voltage_v', 'current_a', 'cell_v', 'fraction'])

        self.create_subscription(BatteryState, 'battery_state',
                                 self._on_battery, 10)
        self.create_timer(float(self.get_parameter('period_seconds').value),
                          self._tick)

        self.get_logger().info(
            f'Logging to {path}. Pack floor {self.floor:.2f} V '
            f'({self.cells}S). Leave the robot powered and untouched.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_battery(self, msg: BatteryState):
        self.voltage = msg.voltage
        self.current = msg.current

    def _tick(self):
        if self.voltage is None:
            self.get_logger().warn(
                'No battery_state received. Publish pack voltage, or record '
                'it manually alongside this log.', once=True)
            return

        elapsed = self._now() - self.started
        cell = self.voltage / self.cells
        fraction = max(0.0, min(1.0, (self.voltage - self.floor) /
                                (self.full - self.floor)))

        self.writer.writerow([f'{elapsed:.1f}', f'{self.voltage:.3f}',
                              '' if self.current is None else f'{self.current:.3f}',
                              f'{cell:.3f}', f'{fraction:.3f}'])
        self.csv_file.flush()

        if self.voltage <= self.floor and not self.warned:
            self.warned = True
            self.get_logger().error(
                f'Pack reached the floor after {elapsed / 60.0:.1f} min. '
                f'Stop the trial and recharge.')
        elif fraction < 0.2 and not self.warned:
            self.get_logger().warn(f'Below 20 % at {elapsed / 60.0:.1f} min.')

    def destroy_node(self):
        self.csv_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EnduranceLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
