"""Convert nav_msgs/Odometry into the odom -> base_link TF transform.

Algorithm:
1. Subscribe to an odometry message, normally `/odom` from the ESP32 firmware.
2. Copy pose position and orientation into a `TransformStamped`.
3. Choose stable frame names from parameters unless `use_msg_frame_ids` is true.
4. Stamp the transform with the Raspberry Pi ROS clock by default, keeping it in
   the same time domain as LiDAR and camera messages.
5. Broadcast the transform so SLAM Toolbox and Nav2 can resolve:
   `map -> odom -> base_link -> sensors`.
"""

# References (full entries: docs/references.md at the repository root):
#   [R13] REP 105, coordinate frames: odom -> base_link frame convention
#   [R10] ROS 2 Jazzy documentation: tf2 broadcaster
# More details: README.md of this package

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcasterNode(Node):
    """Broadcast one TF transform for each incoming odometry sample."""

    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster_node")

        # Topic/frame parameters make the node reusable with future firmware.
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_msg_frame_ids", False)
        self.declare_parameter("use_msg_stamp", False)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.use_msg_frame_ids = bool(self.get_parameter("use_msg_frame_ids").value)
        self.use_msg_stamp = bool(self.get_parameter("use_msg_stamp").value)
        self.broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            20,
        )

    def _odom_callback(self, msg: Odometry) -> None:
        transform = TransformStamped()

        # Pi clock is safer before micro-ROS time sync; message stamp remains optional.
        transform.header.stamp = (
            msg.header.stamp if self.use_msg_stamp else self.get_clock().now().to_msg()
        )
        transform.header.frame_id = (
            msg.header.frame_id
            if self.use_msg_frame_ids and msg.header.frame_id
            else self.odom_frame
        )
        transform.child_frame_id = (
            msg.child_frame_id
            if self.use_msg_frame_ids and msg.child_frame_id
            else self.base_frame
        )
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcasterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
