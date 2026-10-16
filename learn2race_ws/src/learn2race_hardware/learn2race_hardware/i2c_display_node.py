"""ROS adapter for the 0.91 inch SSD1306 I2C OLED display.

text_scale 1 fits status, battery, network and speed on one 4-line screen.
text_scale 2 uses double-size text on two lines that rotate between pages.
A safety fault always takes over, with the reason scrolling.
"""

# References (full entries: docs/references.md at the repository root):
#   [R78] SSD1306 datasheet: SSD1306
# More details: README.md of this package

from __future__ import annotations

import math
import socket
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import Bool, String

from .oled_font import Framebuffer
from .ssd1306_i2c import Ssd1306I2c


class I2cDisplayNode(Node):
    """Render rotating two-line status pages on the SSD1306 OLED."""

    def __init__(self) -> None:
        super().__init__("i2c_display_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 60)
        self.declare_parameter("width", 128)
        self.declare_parameter("height", 32)
        self.declare_parameter("rotate", 0)            # 0 or 180
        self.declare_parameter("text_scale", 1)        # 1: 4 lines, 2: 2 large lines
        self.declare_parameter("refresh_hz", 4.0)
        self.declare_parameter("page_seconds", 3.0)
        self.declare_parameter("odom_timeout_s", 1.0)
        self.declare_parameter("joy_timeout_s", 1.0)
        self.declare_parameter("battery_timeout_s", 3.0)
        self.declare_parameter("dry_run", False)

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.rotate = int(self.get_parameter("rotate").value) % 360
        if self.rotate not in (0, 180):
            raise ValueError("rotate must be 0 or 180 on a 128x32 panel")
        self.scale = int(self.get_parameter("text_scale").value)
        if self.scale not in (1, 2):
            raise ValueError("text_scale must be 1 or 2")
        self.columns = self.width // (6 * self.scale)
        self.page_seconds = max(1.0, float(self.get_parameter("page_seconds").value))
        self.odom_timeout_s = float(self.get_parameter("odom_timeout_s").value)
        self.joy_timeout_s = float(self.get_parameter("joy_timeout_s").value)
        self.battery_timeout_s = float(self.get_parameter("battery_timeout_s").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.display: Optional[Ssd1306I2c] = None
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.last_joy_time = None
        self.last_odom_time = None
        self.last_battery_time = None
        self.battery = None
        self.fault = False
        # None until /safety/ready is heard: robots without that topic (B)
        # are then judged by the fault stream alone instead of waiting forever.
        self.safety_ready = None
        self.last_safety_time = None
        self.fault_reason = ""
        self.next_retry = 0.0
        self.tick = 0
        self.page = 0
        self.page_started = time.monotonic()
        self.ip = self._ip_address()

        self._init_display()

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "/battery_state", self._battery_callback, qos_profile_sensor_data)
        self.create_subscription(Bool, "/safety/fault", self._fault_callback, 10)
        self.create_subscription(Bool, "/safety/ready", self._ready_callback, 10)
        self.create_subscription(String, "/safety/fault_reason", self._reason_callback, 10)

        refresh_hz = max(1.0, float(self.get_parameter("refresh_hz").value))
        self.create_timer(1.0 / refresh_hz, self._update)

    def _init_display(self) -> None:
        if self.dry_run:
            self.get_logger().warn("I2C display dry_run=true; not touching I2C.")
            return

        address = self._parse_address(self.get_parameter("i2c_address").value)
        bus = int(self.get_parameter("i2c_bus").value)

        try:
            self.display = Ssd1306I2c(bus, address, self.width, self.height)
            self.get_logger().info(
                f"Started SSD1306 I2C display {self.width}x{self.height} "
                f"on bus {bus}, address 0x{address:02X}."
            )
        except OSError as exc:
            self.next_retry = time.monotonic() + 5.0
            self.get_logger().error(
                "Could not start I2C display. Check wiring, I2C enablement, and address "
                f"with `i2cdetect -y {bus}`. Retrying in 5 seconds. Error: {exc}"
            )
        except (RuntimeError, ValueError) as exc:
            self.dry_run = True
            self.get_logger().error(f"Could not start I2C display: {exc}")

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _joy_callback(self, msg: Joy) -> None:
        del msg
        self.last_joy_time = self.get_clock().now()

    def _odom_callback(self, msg: Odometry) -> None:
        del msg
        self.last_odom_time = self.get_clock().now()

    def _battery_callback(self, msg: BatteryState) -> None:
        self.battery = msg
        self.last_battery_time = self.get_clock().now()

    def _fault_callback(self, msg: Bool) -> None:
        self.fault = bool(msg.data)
        self.last_safety_time = self.get_clock().now()

    def _ready_callback(self, msg: Bool) -> None:
        self.safety_ready = bool(msg.data)

    def _reason_callback(self, msg: String) -> None:
        self.fault_reason = msg.data

    @staticmethod
    def _age(now, stamp):
        return float("inf") if stamp is None else (now - stamp).nanoseconds / 1e9

    def _scroll(self, text: str) -> str:
        """Fit text to one line, scrolling it one character per refresh."""
        if len(text) <= self.columns:
            return text
        loop = text + "   "
        offset = self.tick % len(loop)
        return (loop + loop)[offset:offset + self.columns]

    # ------------------------------------------------------------------ pages

    def _status_page(self, now, state):
        if state == "WAIT" and self.fault_reason:
            return state, self._scroll(self.fault_reason)
        if state == "DRIVE":
            return state, f"{self.last_cmd.linear.x:+.2f}m/s"
        odom_ok = self._age(now, self.last_odom_time) <= self.odom_timeout_s
        joy_ok = self._age(now, self.last_joy_time) <= self.joy_timeout_s
        return state, f"E:{'OK' if odom_ok else '--'} J:{'OK' if joy_ok else '--'}"

    def _battery_page(self, now):
        msg = self.battery
        if msg is None or self._age(now, self.last_battery_time) > self.battery_timeout_s \
                or not math.isfinite(msg.voltage):
            return "BAT", "no data"
        second = f"{msg.current:+.2f}A" if math.isfinite(msg.current) else ""
        if math.isfinite(msg.percentage):
            second = f"{second} {msg.percentage * 100:.0f}%".strip()
        return f"BAT {msg.voltage:.2f}V", second

    def _network_page(self):
        if self.ip is None:
            return "NETWORK", "offline"
        octets = self.ip.split(".")
        return ".".join(octets[:2]) + ".", ".".join(octets[2:])

    def _update(self) -> None:
        now = self.get_clock().now()
        self.tick += 1
        cmd_fresh = self._age(now, self.last_cmd_time) <= 0.7
        moving = cmd_fresh and (abs(self.last_cmd.linear.x) > 0.01
                                or abs(self.last_cmd.angular.z) > 0.01)
        odom_ok = self._age(now, self.last_odom_time) <= self.odom_timeout_s
        safety_stale = self._age(now, self.last_safety_time) > 1.0

        state = ("FAULT" if self.fault else
                 "WAIT" if not odom_ok or self.safety_ready is False or safety_stale
                 else "DRIVE" if moving else "READY")
        if self.scale == 1:
            if self.tick % 40 == 0:       # about every 10 s at 4 Hz
                self.ip = self._ip_address()
            lines = self._compact_screen(now, state)
        elif self.fault:
            lines = ("FAULT", self._scroll(self.fault_reason or "see safety monitor"))
        else:
            if time.monotonic() - self.page_started >= self.page_seconds:
                self.page = (self.page + 1) % 3
                self.page_started = time.monotonic()
                if self.page == 2:
                    self.ip = self._ip_address()
            lines = (self._status_page(now, state), self._battery_page(now),
                     self._network_page())[self.page]

        if not self.dry_run and self.display is None and time.monotonic() >= self.next_retry:
            self._init_display()
        self._draw_lines(lines)

    def _compact_screen(self, now, state):
        _, status = self._status_page(now, "READY")
        battery, charge = self._battery_page(now)
        if state in ("FAULT", "WAIT") and self.fault_reason:
            last = self._scroll(self.fault_reason)
        else:
            last = f"V:{self.last_cmd.linear.x:+.2f} W:{self.last_cmd.angular.z:+.2f}"
        return (f"{state:<6} {status}",
                f"{battery} {charge}",
                f"IP {self.ip}" if self.ip else "IP offline",
                last)

    def _draw_lines(self, lines) -> None:
        frame = Framebuffer(self.width, self.height)
        for row, line in enumerate(lines):
            frame.text(0, row * 8 * self.scale, line[:self.columns], scale=self.scale)
        if self.dry_run or self.display is None:
            return
        data = frame.rotated_180() if self.rotate == 180 else bytes(frame.buffer)
        try:
            self.display.show(data)
        except OSError as exc:
            self.get_logger().error(f"OLED disconnected: {exc}; retrying in 5 seconds")
            self.display.close()
            self.display = None
            self.next_retry = time.monotonic() + 5.0

    @staticmethod
    def _ip_address() -> Optional[str]:
        """Return the IPv4 address used for the default route, if any."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect(("8.8.8.8", 80))    # UDP connect sends no packet
                return sock.getsockname()[0]
        except OSError:
            return None

    @staticmethod
    def _parse_address(value) -> int:
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def destroy_node(self) -> bool:
        if self.display is not None:
            self.display.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = I2cDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
