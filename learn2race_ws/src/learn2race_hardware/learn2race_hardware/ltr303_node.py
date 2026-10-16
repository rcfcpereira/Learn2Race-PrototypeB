#!/usr/bin/env python3
"""LTR-303 ambient light sensor (I2C, fixed address 0x29).

Publishes visible+IR and IR channels. Used on Prototype A to adapt the HSV
segmentation thresholds of the camera line follower to venue lighting, and as a
sanity check on the start detector.
"""

# References (full entries: docs/references.md at the repository root):
#   [R67] LTR-303ALS-01 datasheet: LTR-303 registers and gain
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Illuminance
from std_msgs.msg import UInt32

try:
    from smbus2 import SMBus
except ImportError:                                   # pragma: no cover
    SMBus = None

REG_CONTROL = 0x80
REG_MEAS_RATE = 0x85
REG_DATA_CH1_0 = 0x88          # IR
REG_DATA_CH0_0 = 0x8A          # visible + IR


class Ltr303Node(Node):

    def __init__(self):
        super().__init__('ltr303')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x29)
        self.declare_parameter('publish_hz', 10.0)

        if SMBus is None:
            raise RuntimeError('smbus2 is required for the LTR-303 node')
        self.bus = SMBus(int(self.get_parameter('i2c_bus').value))
        self.address = int(self.get_parameter('address').value)

        self.bus.write_byte_data(self.address, REG_CONTROL, 0x01)   # active mode
        self.bus.write_byte_data(self.address, REG_MEAS_RATE, 0x13) # 100 ms

        self.pub_vis = self.create_publisher(UInt32, 'light/visible_plus_ir_raw', 10)
        self.pub_ir = self.create_publisher(UInt32, 'light/ir_raw', 10)
        self.pub_lux = self.create_publisher(Illuminance, 'light/illuminance', 10)

        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

    def _read_word(self, reg):
        low = self.bus.read_byte_data(self.address, reg)
        high = self.bus.read_byte_data(self.address, reg + 1)
        return (high << 8) | low

    def _tick(self):
        ir = self._read_word(REG_DATA_CH1_0)
        vis = self._read_word(REG_DATA_CH0_0)
        self.pub_ir.publish(UInt32(data=int(ir)))
        self.pub_vis.publish(UInt32(data=int(vis)))

        msg = Illuminance()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.illuminance = float(max(0, vis - ir))
        msg.variance = 0.0
        self.pub_lux.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Ltr303Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
