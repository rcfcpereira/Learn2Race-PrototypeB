"""Republish the LD06 scan with a fixed number of beams (scan -> scan_fixed).

The LD06 driver sends 451-454 points per turn. SLAM Toolbox fixes the beam count from its first
scan and drops every scan with another count ("LaserRangeScan contains 454 range readings,
expected 453"): about 37 % of the scans on Prototype B (15 October 2026). Each point goes to the
nearest of `beams` equal bins over 360 deg (the closest range wins when two fall in one bin); empty
bins are +inf. Header, scan time and range limits are copied.
"""
# References (full entries: docs/references.md at the repository root):
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox
#   [R79] LDROBOT STL-06P lidar, ldlidar_stl_ros2: lidar
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFixedNode(Node):
    """Resample each LaserScan to a constant number of beams."""

    def __init__(self) -> None:
        super().__init__("scan_fixed")
        self.declare_parameter("beams", 453)
        self.n = int(self.get_parameter("beams").value)
        self.inc = 2 * math.pi / self.n
        self.pub = self.create_publisher(LaserScan, "scan_fixed", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, m: LaserScan) -> None:
        ranges = [math.inf] * self.n
        inten = [0.0] * self.n
        has_i = len(m.intensities) == len(m.ranges)
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max):
                continue
            k = int(round(((m.angle_min + i * m.angle_increment) % (2 * math.pi)) / self.inc)) % self.n
            if r < ranges[k]:
                ranges[k] = r
                if has_i:
                    inten[k] = m.intensities[i]
        out = LaserScan()
        out.header = m.header
        out.angle_min, out.angle_max, out.angle_increment = 0.0, self.inc * (self.n - 1), self.inc
        out.time_increment = m.scan_time / self.n if m.scan_time > 0 else 0.0
        out.scan_time, out.range_min, out.range_max = m.scan_time, m.range_min, m.range_max
        out.ranges = ranges
        out.intensities = inten if has_i else []
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanFixedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
