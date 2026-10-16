#!/usr/bin/env python3
"""TOF050C time-of-flight ranging module (I2C mode).

The TOF050C is the 50 cm variant of the TOF0xxC family. The module exposes a
distance register in millimetres; the 200 cm and 400 cm variants share the
protocol and differ only in range, so max_range is a parameter.
"""

# References (full entries: docs/references.md at the repository root):
#   [R72] VL6180X datasheet, TOF050C module: TOF050C module
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    from smbus2 import SMBus
except ImportError:                                   # pragma: no cover
    SMBus = None

REG_DISTANCE = 0x00


class Tof050cNode(Node):

    def __init__(self):
        super().__init__('tof050c')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x52)
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('min_range_m', 0.01)
        self.declare_parameter('max_range_m', 0.50)
        self.declare_parameter('field_of_view_rad', 0.44)
        self.declare_parameter('publish_hz', 20.0)

        if SMBus is None:
            raise RuntimeError('smbus2 is required for the TOF050C node')
        self.bus = SMBus(int(self.get_parameter('i2c_bus').value))
        self.address = int(self.get_parameter('address').value)

        self.pub = self.create_publisher(Range, 'tof/range', 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

    def _tick(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, REG_DISTANCE, 2)
        except OSError as exc:
            # A missing sensor fails every cycle; one line per 5 s is enough.
            self.get_logger().warn(f'TOF050C read failed: {exc}', throttle_duration_sec=5.0)
            return

        millimetres = (data[0] << 8) | data[1]

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self.get_parameter('field_of_view_rad').value)
        msg.min_range = float(self.get_parameter('min_range_m').value)
        msg.max_range = float(self.get_parameter('max_range_m').value)
        msg.range = millimetres / 1000.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Tof050cNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
