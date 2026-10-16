#!/usr/bin/env python3
"""Grove Ultrasonic Ranger on a Raspberry Pi GPIO pin.

The Grove ranger uses a single signal line for both trigger and echo, so the pin
is switched from output to input between the pulse and the measurement. Readings
are rejected when the echo never arrives, which happens on absorbent or steeply
angled surfaces; this complements the optical sensors rather than duplicating
them.
"""

# References (full entries: docs/references.md at the repository root):
#   [R74] Seeed Grove module wiki: Grove Ultrasonic Ranger
#   [R65] Raspberry Pi 4/5 documentation: Pi GPIO
# More details: README.md of this package

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import RPi.GPIO as GPIO
except ImportError:                                   # pragma: no cover
    GPIO = None

SPEED_OF_SOUND = 343.0


class UltrasonicNode(Node):

    def __init__(self):
        super().__init__('ultrasonic')
        self.declare_parameter('gpio_pin', 17)
        self.declare_parameter('frame_id', 'ultrasonic_link')
        self.declare_parameter('min_range_m', 0.02)
        self.declare_parameter('max_range_m', 3.5)
        self.declare_parameter('timeout_seconds', 0.03)
        self.declare_parameter('publish_hz', 10.0)

        if GPIO is None:
            raise RuntimeError('RPi.GPIO is required for the ultrasonic node')

        self.pin = int(self.get_parameter('gpio_pin').value)
        GPIO.setmode(GPIO.BCM)

        self.pub = self.create_publisher(Range, 'ultrasonic/range', 10)
        self.create_timer(1.0 / float(self.get_parameter('publish_hz').value),
                          self._tick)

    def _measure(self):
        timeout = float(self.get_parameter('timeout_seconds').value)

        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)
        time.sleep(2e-6)
        GPIO.output(self.pin, GPIO.HIGH)
        time.sleep(10e-6)
        GPIO.output(self.pin, GPIO.LOW)
        GPIO.setup(self.pin, GPIO.IN)

        deadline = time.time() + timeout
        while GPIO.input(self.pin) == 0:
            if time.time() > deadline:
                return None
        start = time.time()

        while GPIO.input(self.pin) == 1:
            if time.time() > deadline:
                return None
        return (time.time() - start) * SPEED_OF_SOUND / 2.0

    def _tick(self):
        distance = self._measure()
        if distance is None:
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = float(self.get_parameter('min_range_m').value)
        msg.max_range = float(self.get_parameter('max_range_m').value)
        msg.range = float(distance)
        self.pub.publish(msg)

    def destroy_node(self):
        if GPIO is not None:
            GPIO.cleanup(self.pin)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
