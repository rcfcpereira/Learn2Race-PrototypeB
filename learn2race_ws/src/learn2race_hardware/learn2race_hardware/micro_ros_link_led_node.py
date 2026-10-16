"""Reflect ESP32 micro-ROS connection health on the Grove RGB Stick.

The ESP32 only publishes `/odom` once its micro-ROS session with the agent is
fully established (RobotApp::publishOdometryIfDue runs from the Connected
state's control timer). There is no other Pi-side signal for "is the
firmware actually talking to us right now" - the agent's own log is the only
other source, and it is not glanceable on the robot itself. This node turns
`/odom` freshness into a status color on the existing RGB stick so the link
state is visible without a laptop:
    stale/never received -> blinking red  (searching / not connected)
    received recently    -> solid green   (session established)

It republishes continuously (well under grove_rgb_stick_node's
manual_hold_s) so it holds the LEDs as a dedicated status display for as
long as this node runs, overriding that node's own idle/drive patterns.
"""

# References (full entries: docs/references.md at the repository root):
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS agent session
#   [R75] WS2813 datasheet, Linux spidev, rpi_ws281x: WS2813 stick
# More details: README.md of this package

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import ColorRGBA


class MicroRosLinkLedNode(Node):
    """Publish a connection-status color to /leds/set_color from /odom freshness."""

    def __init__(self) -> None:
        super().__init__("micro_ros_link_led_node")

        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("blink_period_s", 0.15)

        self.stale_timeout_s = self.get_parameter("stale_timeout_s").value
        self.blink_period_s = self.get_parameter("blink_period_s").value

        self.last_odom_time = None  # None until the first /odom message ever arrives
        self.blink_on = False

        self.color_pub = self.create_publisher(ColorRGBA, "/leds/set_color", 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_timer(self.blink_period_s, self._update)

    def _odom_callback(self, _msg: Odometry) -> None:
        self.last_odom_time = self.get_clock().now()

    def _update(self) -> None:
        now = self.get_clock().now()
        age_s = None
        if self.last_odom_time is not None:
            age_s = (now - self.last_odom_time).nanoseconds / 1e9

        connected = age_s is not None and age_s <= self.stale_timeout_s
        if connected:
            self._publish(0.0, 1.0, 0.0)  # solid green: session established
            return

        # Not connected: blink red so it reads as "searching", not "broken".
        self.blink_on = not self.blink_on
        if self.blink_on:
            self._publish(1.0, 0.0, 0.0)
        else:
            self._publish(0.0, 0.0, 0.0)

    def _publish(self, r: float, g: float, b: float) -> None:
        msg = ColorRGBA()
        msg.r = r
        msg.g = g
        msg.b = b
        msg.a = 1.0
        self.color_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MicroRosLinkLedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
