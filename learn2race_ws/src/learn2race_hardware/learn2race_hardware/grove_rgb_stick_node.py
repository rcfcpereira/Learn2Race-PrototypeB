"""ROS adapter for the Grove RGB Stick WS2813 status LEDs.

The hardware library handles `rpi_ws281x`; this node maps robot state, joystick
state, and manual color commands into LED patterns.
"""

# References (full entries: docs/references.md at the repository root):
#   [R74] Seeed Grove module wiki: Grove RGB LED Stick
#   [R75] WS2813 datasheet, Linux spidev, rpi_ws281x: WS2813
# More details: README.md of this package; grove_ws2813_rgb_stick.py

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from .grove_ws2813_rgb_stick import GroveWs2813RgbStick, Rgb, SpiWs2813RgbStick
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA


class GroveRgbStickNode(Node):
    """Map ROS command/controller state into LED status patterns."""

    def __init__(self) -> None:
        super().__init__("grove_rgb_stick_node")

        self.declare_parameter("led_count", 10)
        self.declare_parameter("gpio_pin", 21)
        self.declare_parameter("brightness", 64)
        self.declare_parameter("freq_hz", 800000)
        self.declare_parameter("dma", 10)
        self.declare_parameter("invert", False)
        self.declare_parameter("channel", 0)
        # Timeout values decide whether recent `/cmd_vel` and `/joy` data are valid.
        self.declare_parameter("command_timeout_s", 0.7)
        self.declare_parameter("manual_hold_s", 2.0)
        self.declare_parameter("idle_brightness", 0.12)
        self.declare_parameter("dry_run", False)
        # auto: rpi_ws281x, falling back to SPI when the stick is on GPIO10
        # (MOSI), as rpi_ws281x cannot drive a Raspberry Pi 5. spi / pwm force one.
        self.declare_parameter("backend", "auto")

        self.led_count = self.get_parameter("led_count").value
        self.gpio_pin = self.get_parameter("gpio_pin").value
        self.brightness = self.get_parameter("brightness").value
        self.command_timeout_s = self.get_parameter("command_timeout_s").value
        self.manual_hold_s = self.get_parameter("manual_hold_s").value
        self.idle_brightness = self.get_parameter("idle_brightness").value
        self.dry_run = self.get_parameter("dry_run").value

        self.strip = None
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_joy = Joy()
        self.last_joy_time = self.get_clock().now()
        # Manual color briefly overrides automatic drive/idle patterns.
        self.manual_color: Optional[Rgb] = None
        self.manual_until = self.get_clock().now()
        self.frame = 0
        self.last_pixels: List[Rgb] = []

        self._init_strip()

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.create_subscription(ColorRGBA, "/leds/set_color", self._manual_color_callback, 10)
        self.create_timer(0.05, self._update)

    def _init_strip(self) -> None:
        if self.dry_run:
            self.get_logger().warn("RGB stick dry_run=true; not touching GPIO.")
            return

        backend = str(self.get_parameter("backend").value)
        pwm_error = None
        if backend in ("auto", "pwm"):
            try:
                strip = GroveWs2813RgbStick(
                    led_count=self.led_count,
                    gpio_pin=self.gpio_pin,
                    brightness=self.brightness,
                    freq_hz=self.get_parameter("freq_hz").value,
                    dma=self.get_parameter("dma").value,
                    invert=self.get_parameter("invert").value,
                    channel=self.get_parameter("channel").value,
                )
                strip.begin()
                self.strip = strip
                self.get_logger().info(
                    f"Started Grove RGB Stick (rpi_ws281x): {self.led_count} LEDs "
                    f"on BCM GPIO{self.gpio_pin}."
                )
                return
            except (RuntimeError, OSError) as exc:
                pwm_error = exc

        if backend == "spi" or (backend == "auto" and self.gpio_pin == 10):
            try:
                strip = SpiWs2813RgbStick(self.led_count, brightness=self.brightness)
                strip.begin()
                self.strip = strip
                self.get_logger().info(
                    f"Started Grove RGB Stick over SPI (/dev/spidev0.0, MOSI = GPIO10): "
                    f"{self.led_count} LEDs."
                )
                return
            except (RuntimeError, OSError) as exc:
                pwm_error = f"{pwm_error}; SPI: {exc}" if pwm_error else exc

        self.dry_run = True
        self.get_logger().error(
            "Could not start the RGB stick; falling back to dry-run. On a "
            "Raspberry Pi 5 wire DIN to BCM GPIO10 (MOSI), enable SPI and use "
            f"backend:=spi. Error: {pwm_error}"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def _joy_callback(self, msg: Joy) -> None:
        self.last_joy = msg
        self.last_joy_time = self.get_clock().now()

    def _manual_color_callback(self, msg: ColorRGBA) -> None:
        self.manual_color = (
            self._to_byte(msg.r),
            self._to_byte(msg.g),
            self._to_byte(msg.b),
        )
        self.manual_until = self.get_clock().now() + Duration(seconds=self.manual_hold_s)

    def _update(self) -> None:
        now = self.get_clock().now()
        self.frame += 1

        # Manual override wins first; otherwise the LEDs reflect controller/drive state.
        if self.manual_color is not None and now < self.manual_until:
            self._show_solid(self.manual_color)
            return

        age_s = (now - self.last_cmd_time).nanoseconds / 1e9
        joy_age_s = (now - self.last_joy_time).nanoseconds / 1e9
        linear = float(self.last_cmd.linear.x)
        angular = float(self.last_cmd.angular.z)
        moving = age_s <= self.command_timeout_s and (
            abs(linear) > 0.01 or abs(angular) > 0.01
        )

        if joy_age_s < 0.5 and len(self.last_joy.buttons) > 5 and self.last_joy.buttons[5]:
            self._show_solid((255, 255, 255))
        elif joy_age_s < 0.5 and len(self.last_joy.buttons) > 4 and self.last_joy.buttons[4]:
            self._show_solid((0, 90, 255))
        elif moving and linear > 0.01:
            self._show_direction((0, 180, 45), angular)
        elif moving and linear < -0.01:
            self._show_direction((220, 35, 0), angular)
        elif moving:
            self._show_direction((255, 120, 0), angular)
        else:
            self._show_idle()

    def _show_idle(self) -> None:
        # A soft blue pulse shows the node is alive while motors are idle.
        pulse = (math.sin(self.frame * 0.18) + 1.0) * 0.5
        value = int(20 + 70 * pulse * self.idle_brightness)
        self._show_solid((0, 0, value))

    def _show_direction(self, base: Rgb, angular: float) -> None:
        # Orange half-strip highlights the turning side while the base color
        # communicates forward, reverse, or in-place rotation.
        pixels = [base for _ in range(self.led_count)]
        if angular > 0.15:
            pixels[: self.led_count // 2] = [(255, 140, 0)] * (self.led_count // 2)
        elif angular < -0.15:
            pixels[self.led_count // 2 :] = [(255, 140, 0)] * (
                self.led_count - self.led_count // 2
            )
        self._show_pixels(pixels)

    def _show_solid(self, color: Rgb) -> None:
        self._show_pixels([color] * self.led_count)

    def _show_pixels(self, pixels: List[Rgb]) -> None:
        if pixels == self.last_pixels:
            return
        self.last_pixels = list(pixels)

        if self.dry_run or self.strip is None:
            return

        try:
            self.strip.show_pixels(pixels)
        except OSError as exc:
            # A status LED must never take the node down with it.
            self.get_logger().error(f"RGB stick write failed, switching to dry-run: {exc}")
            self.strip = None
            self.dry_run = True

    @staticmethod
    def _to_byte(value: float) -> int:
        return max(0, min(255, int(value * 255.0)))

    def destroy_node(self) -> bool:
        self._show_solid((0, 0, 0))
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GroveRgbStickNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
