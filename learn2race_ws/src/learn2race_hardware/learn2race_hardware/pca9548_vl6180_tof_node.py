"""ROS adapter for three VL6180X ToF sensors behind a PCA9548 mux."""

# References (full entries: docs/references.md at the repository root):
#   [R77] PCA9548A / TCA9548A datasheet: PCA9548A
#   [R72] VL6180X datasheet, TOF050C module: VL6180X
# More details: README.md of this package

from dataclasses import dataclass
from math import inf
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from learn2race_bringup.pca9548 import Pca9548
from learn2race_bringup.vl6180x import Vl6180x

try:
    from smbus2 import SMBus
except ImportError:  # Hardware library is only expected on the Raspberry Pi.
    SMBus = None


@dataclass
class TofSensor:
    """Runtime binding between mux channel, ROS frame/topic, and sensor driver."""

    channel: int
    frame_id: str
    publisher: object
    sensor: Vl6180x


class Pca9548Vl6180TofNode(Node):
    """Publish Range messages from identical VL6180X sensors behind a mux."""

    def __init__(self) -> None:
        super().__init__("pca9548_vl6180_tof_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("mux_address", 112)
        self.declare_parameter("sensor_address", 41)
        # Matching arrays map mux channels to ROS frames and topics.
        self.declare_parameter("channels", [0, 1, 2])
        self.declare_parameter(
            "frame_ids", ["tof_front_link", "tof_left_link", "tof_right_link"]
        )
        self.declare_parameter("topics", ["/tof/front", "/tof/left", "/tof/right"])
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("read_timeout_s", 0.05)
        self.declare_parameter("field_of_view_rad", 0.44)
        self.declare_parameter("min_range_m", 0.005)
        self.declare_parameter("max_range_m", 0.20)
        self.declare_parameter("dry_run", False)
        # Per-channel range offset (m), subtracted from each reading; matches `channels`.
        # The VL6180X part-to-part offset: measured against a tape on 15 Oct 2026 (test 4.7).
        self.declare_parameter("range_offsets_m", [0.0, 0.0, 0.0])

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.read_timeout_s = float(self.get_parameter("read_timeout_s").value)
        self.field_of_view_rad = float(self.get_parameter("field_of_view_rad").value)
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.offsets = dict(zip(
            [int(v) for v in self.get_parameter("channels").value],
            [float(v) for v in self.get_parameter("range_offsets_m").value]))
        self.last_error_log_time = self.get_clock().now()

        self.bus = None
        self.mux: Optional[Pca9548] = None
        self.sensors: List[TofSensor] = []
        # Channels whose sensor did not answer at start-up, retried every 5 s.
        self.pending: List[tuple] = []
        self.start_failures = {}          # channel -> failed start attempts
        self.sensor_address = 0x29
        self.last_retry_s = 0.0

        self._init_sensors()

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self._publish_ranges)

    def _init_sensors(self) -> None:
        if self.dry_run:
            self.get_logger().warn("PCA9548/VL6180X dry_run=true; not touching I2C.")
            return

        if SMBus is None:
            self.dry_run = True
            self.get_logger().warn(
                "Python package smbus2 is not installed; running ToF node in dry-run mode."
            )
            return

        channels = [int(value) for value in self.get_parameter("channels").value]
        frame_ids = [str(value) for value in self.get_parameter("frame_ids").value]
        topics = [str(value) for value in self.get_parameter("topics").value]
        if not (len(channels) == len(frame_ids) == len(topics)):
            self.dry_run = True
            self.get_logger().error(
                "ToF config must have the same number of channels, frame_ids, and topics."
            )
            return

        bus_id = int(self.get_parameter("i2c_bus").value)
        mux_address = self._parse_address(self.get_parameter("mux_address").value)
        sensor_address = self._parse_address(self.get_parameter("sensor_address").value)

        self.sensor_address = sensor_address
        try:
            # Only a missing multiplexer disables the node. One sensor that does
            # not answer must not take the others down with it.
            self.bus = SMBus(bus_id)
            self.mux = Pca9548(self.bus, mux_address)
            self.mux.disable_all()
        except (OSError, RuntimeError, ValueError) as exc:
            self.dry_run = True
            self.get_logger().error(
                "Could not start PCA9548/VL6180X ToF sensors. Check wiring, I2C "
                f"enablement, mux address, and channels with `i2cdetect -y {bus_id}`. "
                f"Falling back to dry-run. Error: {exc}"
            )
            return

        for channel, frame_id, topic in zip(channels, frame_ids, topics):
            if not self._start_channel(channel, frame_id, topic):
                self.pending.append((channel, frame_id, topic))
        self.mux.disable_all()

    def _start_channel(self, channel: int, frame_id: str, topic: str) -> bool:
        try:
            self.mux.select_channel(channel)
            sensor = Vl6180x(self.bus, self.sensor_address)
            model_id = sensor.begin()
        except (OSError, RuntimeError, ValueError) as exc:
            failures = self.start_failures.get(channel, 0) + 1
            self.start_failures[channel] = failures
            # Retries run every 5 s; report the first failure, then once a minute.
            if failures == 1 or failures % 12 == 0:
                self.get_logger().error(
                    f"VL6180X on mux channel {channel} ({topic}) did not start "
                    f"({failures} attempts): {exc}. The other channels keep "
                    "running; retrying every 5 s. A sensor that does not even "
                    "answer its model-id read is a wiring, CE-pin or supply fault."
                )
            return False
        if self.start_failures.pop(channel, 0):
            self.get_logger().info(f"VL6180X on mux channel {channel} recovered.")
        self.sensors.append(
            TofSensor(
                channel=channel,
                frame_id=frame_id,
                publisher=self.create_publisher(Range, topic, 10),
                sensor=sensor,
            )
        )
        self.get_logger().info(
            f"Started VL6180X ToF on mux channel {channel}, sensor address "
            f"0x{self.sensor_address:02X}, MODEL_ID=0x{model_id:02X}, "
            f"topic={topic}, frame={frame_id}."
        )
        return True

    def _retry_pending(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if not self.pending or now_s - self.last_retry_s < 5.0:
            return
        self.last_retry_s = now_s
        self.pending = [entry for entry in self.pending if not self._start_channel(*entry)]

    def _publish_ranges(self) -> None:
        if self.dry_run or self.mux is None:
            return

        self._retry_pending()
        stamp = self.get_clock().now().to_msg()
        for tof in self.sensors:
            try:
                # Only one identical 0x29 VL6180X is visible after selecting a channel.
                self.mux.select_channel(tof.channel)
                reading = tof.sensor.read_range(self.read_timeout_s)
                range_m = reading.distance_mm / 1000.0 - self.offsets.get(tof.channel, 0.0)
                # Invalid or out-of-window readings become +inf, the ROS Range
                # convention for "no obstacle measured in this sensor cone".
                if reading.status != 0 or range_m < self.min_range_m:
                    range_m = inf
                elif range_m > self.max_range_m:
                    range_m = inf
                tof.publisher.publish(self._range_msg(stamp, tof.frame_id, range_m))
            except (OSError, TimeoutError) as exc:
                self._log_read_error(tof.channel, exc)
                tof.publisher.publish(self._range_msg(stamp, tof.frame_id, inf))

        self.mux.disable_all()

    def _range_msg(self, stamp, frame_id: str, range_m: float) -> Range:
        msg = Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.field_of_view_rad
        msg.min_range = self.min_range_m
        msg.max_range = self.max_range_m
        msg.range = range_m
        return msg

    def _log_read_error(self, channel: int, exc: Exception) -> None:
        now = self.get_clock().now()
        elapsed_s = (now - self.last_error_log_time).nanoseconds / 1e9
        if elapsed_s >= 2.0:
            self.last_error_log_time = now
            self.get_logger().warn(f"Could not read VL6180X channel {channel}: {exc}")

    @staticmethod
    def _parse_address(value) -> int:
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def destroy_node(self) -> bool:
        if self.mux is not None:
            self.mux.disable_all()
        if self.bus is not None:
            self.bus.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Pca9548Vl6180TofNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
