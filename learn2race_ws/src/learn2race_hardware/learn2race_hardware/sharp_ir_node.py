#!/usr/bin/env python3
"""Sharp GP2Y0A21 analogue distance sensor, read through an ADS1115 (fitted on Prototype B) or a PCF8591 ADC.

The response is strongly non-linear and is inverted before use: distance is
approximately linear in the reciprocal of the output voltage over the usable
10-80 cm span. Below 10 cm the curve folds back and the reading is ambiguous,
so short readings are reported as out of range rather than as a small distance.
This behaviour makes the sensor a useful teaching example of characterisation
and of the difference between a raw reading and a calibrated measurement.
"""

# References (full entries: docs/references.md at the repository root):
#   [R73] Sharp GP2Y0A21YK0F datasheet: GP2Y0A21 response curve
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: PCF8591
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from .ads1115 import Ads1115

try:
    from smbus2 import SMBus
except ImportError:                                   # pragma: no cover
    SMBus = None


class SharpIrNode(Node):

    def __init__(self):
        super().__init__('sharp_ir')
        self.declare_parameter('i2c_bus', 1)
        # Prototype B carries an ADS1115 at 0x48 (identified 2026-10-15 by its config register,
        # 0x8583 at power-up); the PCF8591 path is kept for the reserved part.
        self.declare_parameter('adc', 'ads1115')         # ads1115 | pcf8591
        self.declare_parameter('address', 0x48)
        self.declare_parameter('channel', 0)
        self.declare_parameter('vref', 3.3)
        self.declare_parameter('frame_id', 'sharp_ir_link')
        self.declare_parameter('min_range_m', 0.10)
        self.declare_parameter('max_range_m', 0.80)
        self.declare_parameter('coeff_a', 27.86)         # d[cm] = a * V^b
        self.declare_parameter('coeff_b', -1.15)
        self.declare_parameter('publish_hz', 20.0)

        if SMBus is None:
            raise RuntimeError('smbus2 is required for the Sharp IR node')
        self.bus = SMBus(int(self.get_parameter('i2c_bus').value))
        self.address = int(self.get_parameter('address').value)
        self.channel = int(self.get_parameter('channel').value)
        self.adc = str(self.get_parameter('adc').value).lower()
        self.ads = Ads1115(self.bus, self.address) if self.adc == 'ads1115' else None

        self.pub = self.create_publisher(Range, 'sharp_ir/range', 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

    def _read_raw(self):
        self.bus.write_byte(self.address, 0x40 | (self.channel & 0x03))
        self.bus.read_byte(self.address)          # discard stale conversion
        return self.bus.read_byte(self.address)

    def _read_volts(self):
        if self.ads is not None:
            return self.ads.read_volts(self.channel, full_scale=4.096)
        return (self._read_raw() / 255.0) * float(self.get_parameter('vref').value)

    def _tick(self):
        try:
            volts = self._read_volts()
        except OSError as exc:
            self.get_logger().warn(f'{self.adc.upper()} read failed: {exc}')
            return

        min_r = float(self.get_parameter('min_range_m').value)
        max_r = float(self.get_parameter('max_range_m').value)

        if volts <= 0.05:
            distance = max_r + 1.0                     # nothing in range
        else:
            a = float(self.get_parameter('coeff_a').value)
            b = float(self.get_parameter('coeff_b').value)
            distance = (a * (volts ** b)) / 100.0

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.08
        msg.min_range = min_r
        msg.max_range = max_r
        msg.range = float(distance)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SharpIrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
