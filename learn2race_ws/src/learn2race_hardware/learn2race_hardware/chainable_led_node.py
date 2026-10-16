"""ROS adapter for the Grove Chainable RGB LED (P9813) on Prototype B's Raspberry Pi 5 GPIO.

Replaces the WS2813 RGB stick, which moved to Prototype A on 2026-10-01. The P9813 takes a
clock and a data line (two GPIOs, driven here bit by bit) and needs no timing beyond edge
order, so it works from user space on the Pi 5. The node shows the colour last received on
leds/set_color (the same topic the stick used, fed by micro_ros_link_led_node and the
competition LED node) and falls back to a dim idle colour when nothing arrives.

Parameters: clock_pin / data_pin (BCM numbers; Grove port D5 = 5 and 6), gpiochip (-1 = find
the Pi's header chip), led_count (chained modules), brightness (0-1), idle_color, hold_s.
"""
# References (full entries: docs/references.md at the repository root):
#   [R76] P9813 datasheet / Seeed Grove Chainable RGB LED: frame format
#   [R77] lgpio and libgpiod: Linux GPIO character device access on the Raspberry Pi 5
# More details: README.md of this package

from __future__ import annotations

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA


class GpioPair:
    """Two output lines on the Pi's GPIO header: lgpio first, libgpiod (v2) as a fallback."""

    def __init__(self, chip: int, pins: tuple[int, int]):
        self.pins = pins
        self.backend = None
        try:
            import lgpio
            # Auto: the chip labelled pinctrl-* carries the 40-pin header (pinctrl-rp1 on the
            # Pi 5, gpiochip4 on kernel 6.8; pinctrl-bcm2711 on the Pi 4). Never guess by number.
            chips = [chip] if chip >= 0 else range(8)
            for c in chips:
                try:
                    h = lgpio.gpiochip_open(c)
                    if chip < 0 and "pinctrl" not in str(lgpio.gpio_get_chip_info(h)[-1]):
                        lgpio.gpiochip_close(h)
                        continue
                    for p in pins:
                        lgpio.gpio_claim_output(h, p, 0)
                    self.lg, self.h, self.backend = lgpio, h, f"lgpio gpiochip{c}"
                    return
                except Exception:  # noqa: BLE001 - try the next chip
                    try:
                        lgpio.gpiochip_close(h)
                    except Exception:  # noqa: BLE001
                        pass
        except ImportError:
            pass
        import gpiod
        from gpiod.line import Direction, Value
        chips = [chip] if chip >= 0 else [0, 4]
        last = None
        for c in chips:
            try:
                self.req = gpiod.request_lines(
                    f"/dev/gpiochip{c}", consumer="chainable_led",
                    config={tuple(pins): gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)})
                self.Value, self.backend = Value, f"gpiod gpiochip{c}"
                return
            except Exception as e:  # noqa: BLE001
                last = e
        raise RuntimeError(f"cannot claim GPIO {pins}: {last}")

    def write(self, pin: int, level: int):
        if self.backend.startswith("lgpio"):
            self.lg.gpio_write(self.h, pin, level)
        else:
            self.req.set_value(pin, self.Value.ACTIVE if level else self.Value.INACTIVE)

    def close(self):
        if self.backend and self.backend.startswith("lgpio"):
            self.lg.gpiochip_close(self.h)
        elif self.backend:
            self.req.release()


class P9813:
    """Frame: 32 zero bits, then per LED a flag byte and blue, green, red; then 32 zero bits."""

    def __init__(self, gpio: GpioPair, clock: int, data: int, count: int):
        self.gpio, self.clock, self.data, self.count = gpio, clock, data, count

    def _byte(self, b: int):
        for i in range(7, -1, -1):
            self.gpio.write(self.data, (b >> i) & 1)
            self.gpio.write(self.clock, 0)
            self.gpio.write(self.clock, 1)

    def show(self, colours: list[tuple[int, int, int]]):
        for _ in range(4):
            self._byte(0)
        for r, g, b in colours:
            flag = 0xC0 | ((~b >> 6) & 3) << 4 | ((~g >> 6) & 3) << 2 | ((~r >> 6) & 3)
            for v in (flag, b, g, r):
                self._byte(v & 0xFF)
        for _ in range(4):
            self._byte(0)


class ChainableLedNode(Node):

    def __init__(self):
        super().__init__("chainable_led")
        self.declare_parameter("clock_pin", 5)
        self.declare_parameter("data_pin", 6)
        self.declare_parameter("gpiochip", -1)
        self.declare_parameter("led_count", 1)
        self.declare_parameter("brightness", 1.0)
        self.declare_parameter("idle_color", [0.0, 0.0, 0.08])
        self.declare_parameter("hold_s", 2.0)
        self.declare_parameter("rate_hz", 20.0)
        p = lambda n: self.get_parameter(n).value
        clock, data = int(p("clock_pin")), int(p("data_pin"))
        self.gpio = GpioPair(int(p("gpiochip")), (clock, data))
        self.led = P9813(self.gpio, clock, data, int(p("led_count")))
        self.brightness = max(0.0, min(1.0, float(p("brightness"))))
        self.idle = tuple(float(c) for c in list(p("idle_color"))[:3])
        self.hold_s = float(p("hold_s"))
        self.colour, self.until, self.shown = self.idle, 0.0, None
        self.create_subscription(ColorRGBA, "leds/set_color", self._on_colour, 10)
        self.create_timer(1.0 / float(p("rate_hz")), self._update)
        self.get_logger().info(
            f"Started Grove Chainable RGB LED (P9813) on GPIO clock {clock}, data {data} "
            f"({self.gpio.backend}), {self.led.count} LED(s).")

    def _on_colour(self, msg: ColorRGBA):
        self.colour = (msg.r, msg.g, msg.b)
        self.until = time.monotonic() + self.hold_s

    def _update(self):
        c = self.colour if time.monotonic() < self.until else self.idle
        rgb = tuple(int(round(255 * self.brightness * max(0.0, min(1.0, x)))) for x in c)
        if rgb != self.shown:      # the P9813 holds its colour; send only on a change
            self.led.show([rgb] * self.led.count)
            self.shown = rgb

    def destroy_node(self):
        try:
            self.led.show([(0, 0, 0)] * self.led.count)
            self.gpio.close()
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChainableLedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C under ros2 run / launch
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
