"""Robot side of gamepad_udp.py: receives {"v", "w"} over UDP port 5005 and publishes teleop/cmd_vel.
Publishes only while packets arrive; if they stop, cmd_vel_mux's 0.5 s source timeout stops the robot.
    python3 udp_teleop.py"""
import json
import socket

import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("udp_teleop")
pub = node.create_publisher(Twist, "teleop/cmd_vel", 10)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))
print("udp_teleop listening on 5005", flush=True)
try:
    while rclpy.ok():
        data, _ = sock.recvfrom(256)
        try:
            cmd = json.loads(data)
        except ValueError:
            continue
        msg = Twist()
        msg.linear.x = max(-0.3, min(0.3, float(cmd.get("v", 0))))
        msg.angular.z = max(-1.5, min(1.5, float(cmd.get("w", 0))))
        pub.publish(msg)
except KeyboardInterrupt:
    pass
