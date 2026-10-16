#!/usr/bin/env python3
"""Reflective line-sensor array node (QYF-750 through an ADS7830).

Publishes the calibrated array and the derived lateral error used by the track
controller. The 8-bit resolution of the ADS7830 makes per-channel min/max
calibration essential: raw counts vary with surface reflectivity, sensor height
and venue lighting, so an uncalibrated array will not transfer between rooms.

Topics
------
    line_array/raw          raw counts, one per channel
    line_array/normalised   0.0 (bright) to 1.0 (dark), calibrated
    line_array/error        lateral error, negative left, positive right
    line_array/on_line      whether any channel currently sees the track

Calibration is captured by sweeping the array across the track with
``calibrate`` set true, then storing the resulting bounds in the parameter file.
"""

# References (full entries: docs/references.md at the repository root):
#   [R80] QYF-750 line-sensor array: QYF-750 array
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS7830
# More details: README.md of this package

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray

try:
    from smbus2 import SMBus
except ImportError:                                   # pragma: no cover
    SMBus = None

from .ads7830 import Ads7830


class LineArrayNode(Node):

    def __init__(self):
        super().__init__('line_array')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('address', 0x48)
        self.declare_parameter('channels', 8)
        self.declare_parameter('publish_hz', 100.0)
        self.declare_parameter('calibrate', False)
        self.declare_parameter('calibration_min', [0] * 8)
        self.declare_parameter('calibration_max', [255] * 8)
        self.declare_parameter('on_line_threshold', 0.45)
        self.declare_parameter('filter_alpha', 0.4)
        self.declare_parameter('invert', False)

        self.channels = int(self.get_parameter('channels').value)
        self.calibrating = bool(self.get_parameter('calibrate').value)
        self.cal_min = list(self.get_parameter('calibration_min').value)
        self.cal_max = list(self.get_parameter('calibration_max').value)
        self.alpha = float(self.get_parameter('filter_alpha').value)
        self.invert = bool(self.get_parameter('invert').value)
        self.filtered = [0.0] * self.channels

        if SMBus is None:
            raise RuntimeError('smbus2 is required for the line array node')
        self.bus = SMBus(int(self.get_parameter('i2c_bus').value))
        self.adc = Ads7830(self.bus, int(self.get_parameter('address').value))

        self.pub_raw = self.create_publisher(Int32MultiArray, 'line_array/raw', 10)
        self.pub_norm = self.create_publisher(Float32MultiArray, 'line_array/normalised', 10)
        self.pub_err = self.create_publisher(Float32, 'line_array/error', 10)
        self.pub_on = self.create_publisher(Bool, 'line_array/on_line', 10)

        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

        if self.calibrating:
            self.get_logger().warn(
                'Calibration mode: sweep the array across the track, then copy '
                'the reported bounds into the parameter file.')

    def _normalise(self, index, raw):
        if self.calibrating:
            self.cal_min[index] = min(self.cal_min[index], raw)
            self.cal_max[index] = max(self.cal_max[index], raw)

        low, high = self.cal_min[index], self.cal_max[index]
        span = max(1, high - low)
        value = (raw - low) / span
        value = min(1.0, max(0.0, value))
        return 1.0 - value if self.invert else value

    def _tick(self):
        raw = self.adc.read_all()[:self.channels]

        norm = []
        for i, value in enumerate(raw):
            n = self._normalise(i, value)
            self.filtered[i] = (self.alpha * n) + ((1.0 - self.alpha) * self.filtered[i])
            norm.append(self.filtered[i])

        self.pub_raw.publish(Int32MultiArray(data=[int(v) for v in raw]))
        self.pub_norm.publish(Float32MultiArray(data=norm))

        total = sum(norm)
        on_line = total >= float(self.get_parameter('on_line_threshold').value)
        self.pub_on.publish(Bool(data=bool(on_line)))

        if on_line and total > 0.0:
            centre = (self.channels - 1) / 2.0
            weighted = sum(i * w for i, w in enumerate(norm)) / total
            self.pub_err.publish(Float32(data=float((weighted - centre) / centre)))


def main(args=None):
    rclpy.init(args=args)
    node = LineArrayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
