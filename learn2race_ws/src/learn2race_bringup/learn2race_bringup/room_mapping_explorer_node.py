"""Conservative autonomous explorer for room mapping.

Reads a 2D LaserScan, drives forward in open space and turns away from obstacles, so SLAM Toolbox
can build a first room map without a driver. It is not Nav2: no goals, no global plan.

Commands go to autonomy/cmd_vel through cmd_vel_mux, and the node arms the mux (safety/arm) only
while it is exploring, so the mux timeout and the safety monitor stay in the path. Any non-zero
teleop/cmd_vel (gamepad) makes it yield for override_hold_s: it stops publishing and disarms, the
mux drops the stale autonomy command and passes the gamepad. It disarms and stops after max_run_s.

Obstacle test in the robot frame (x forward, y left), from scan angle + laser_yaw_deg:
  - emergency: any return in front (x > 0) closer than emergency_distance_m -> turn on the spot;
  - blocked: a return in the corridor ahead (|y| < corridor_half_width_m, 0 < x < obstacle_distance_m)
    -> crawl and turn toward the side with more space;
  - otherwise drive, steering gently away from the closer side.
"""
# References (full entries: docs/references.md at the repository root):
#   [R47] Macenski & Jambrecic (2021) SLAM Toolbox
#   [R3]  Learn2Race dissertation, requirements FR/NFR: FR10 command timeout and safe stop

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool


class RoomMappingExplorerNode(Node):
    """Publish slow exploration commands from the lidar, through the command mux."""

    def __init__(self) -> None:
        super().__init__("room_mapping_explorer_node")
        p = self.declare_parameter
        p("scan_topic", "/scan")
        p("cmd_vel_topic", "autonomy/cmd_vel")
        p("arm_topic", "safety/arm")
        p("teleop_topic", "teleop/cmd_vel")
        p("enable_topic", "/mapping/auto_explore_enable")
        # Prototype B's LD06 has 0 deg to the robot's right: laser_yaw_deg -90.
        p("laser_yaw_deg", 0.0)
        p("linear_speed_mps", 0.10)
        p("crawl_speed_mps", 0.04)
        p("turn_speed_radps", 0.55)
        p("obstacle_distance_m", 0.55)
        p("emergency_distance_m", 0.22)
        p("corridor_half_width_m", 0.14)   # half the robot width (94 mm) plus a margin
        p("side_angle_deg", 105.0)
        p("min_range_m", 0.08)              # ignore returns from the robot itself
        p("scan_timeout_s", 1.0)
        p("override_hold_s", 2.0)
        p("max_run_s", 0.0)                 # 0 = no limit
        p("publish_rate_hz", 10.0)
        p("enabled", True)
        # "Wild" mode: every wander_min_s..wander_max_s, a random turn of 30-150 deg to either side;
        # when blocked, turn the less open way with probability random_escape. 0 = off.
        p("wander_min_s", 0.0)
        p("wander_max_s", 0.0)
        p("random_escape", 0.0)
        # mode "bounce" (drive, turn away from obstacles) or "wall_follow" (keep a wall on follow_side
        # at wall_distance_m, arcs instead of spins, a gentle weave, ToF as a close-range guard).
        p("mode", "bounce")
        p("follow_side", "right")
        p("wall_distance_m", 0.40)
        p("kp_distance", 1.2)
        p("kp_angle", 0.8)
        p("max_angular_radps", 0.6)
        p("angular_accel_radps2", 1.2)     # rate limit on the turn command: smooth, less wheel slip
        p("linear_accel_mps2", 0.3)
        p("weave_amplitude_radps", 0.12)   # snake-like sway while cruising; 0 = straight
        p("weave_period_s", 4.0)
        p("tof_topics", ["/tof/front", "/tof/left", "/tof/right"])
        p("tof_block_m", 0.15)             # a ToF reading below this blocks that direction
        p("tof_front_ahead_of_lidar_m", 0.03)  # front ToF x 0.011 vs lidar x -0.020 (sensor_frames_b.yaml)
        # ToF guard, in every mode, after the mode's command: front below tof_stop_m -> no forward motion,
        # turn to the freer side; a side below tof_side_m -> steer away and crawl. Each trigger is logged.
        p("tof_stop_m", 0.10)
        p("tof_side_m", 0.15)
        p("tof_steer_radps", 0.4)
        # front escape: back up escape_back_s at escape_back_mps, then turn (away from the followed wall)
        # for at least escape_turn_s and until the front ToF is clear; the mode's command is ignored meanwhile.
        p("escape_back_s", 0.7)
        p("escape_back_mps", 0.04)
        p("escape_turn_s", 2.5)            # fallback (no odometry): timed turn
        p("escape_straight_s", 2.0)        # fallback: then straight for this long
        # sidestep with the gyro heading (EKF odometry): turn escape_turn_deg away, drive escape_side_m,
        # turn back to the original heading, and carry on past the object
        p("odom_topic", "/odometry/filtered")
        p("escape_turn_deg", 90.0)
        p("escape_side_m", 0.40)            # 0.30 passed a low box with 6 cm to spare (box test 3)
        # after turning back: straight past the object on the gyro heading, until the side ToF facing it
        # has seen it and lost it again (object behind), or escape_pass_m at most
        p("escape_pass_m", 0.80)
        p("heading_kp", 1.5)                # heading hold on the straight legs (rad/s per rad)

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.yaw = math.radians(float(g("laser_yaw_deg")))
        self.linear_speed = float(g("linear_speed_mps"))
        self.crawl_speed = float(g("crawl_speed_mps"))
        self.turn_speed = float(g("turn_speed_radps"))
        self.obstacle_distance = float(g("obstacle_distance_m"))
        self.emergency_distance = float(g("emergency_distance_m"))
        self.half_width = float(g("corridor_half_width_m"))
        self.side_angle = math.radians(float(g("side_angle_deg")))
        self.min_range = float(g("min_range_m"))
        self.scan_timeout_s = float(g("scan_timeout_s"))
        self.override_hold_s = float(g("override_hold_s"))
        self.max_run_s = float(g("max_run_s"))
        self.enabled = bool(g("enabled"))
        self.wander = (float(g("wander_min_s")), float(g("wander_max_s")))
        self.random_escape = float(g("random_escape"))
        self.mode = str(g("mode"))
        self.side = -1.0 if str(g("follow_side")) == "right" else 1.0
        self.wall_distance = float(g("wall_distance_m"))
        self.kp_d, self.kp_a = float(g("kp_distance")), float(g("kp_angle"))
        self.max_w = float(g("max_angular_radps"))
        self.w_accel, self.v_accel = float(g("angular_accel_radps2")), float(g("linear_accel_mps2"))
        self.weave_amp, self.weave_period = float(g("weave_amplitude_radps")), float(g("weave_period_s"))
        self.tof_block = float(g("tof_block_m"))
        self.tof_front_ahead = float(g("tof_front_ahead_of_lidar_m"))
        self.tof_stop, self.tof_side = float(g("tof_stop_m")), float(g("tof_side_m"))
        self.tof_steer = float(g("tof_steer_radps"))
        self.tof_active = {"front": False, "left": False, "right": False}
        self.tof_events = {"front": 0, "left": 0, "right": 0}
        self.esc_back_s, self.esc_back_v = float(g("escape_back_s")), float(g("escape_back_mps"))
        self.esc_turn_s = float(g("escape_turn_s"))
        self.esc_straight_s = float(g("escape_straight_s"))
        self.esc_turn = math.radians(float(g("escape_turn_deg")))
        self.esc_side = float(g("escape_side_m"))
        self.esc_pass = float(g("escape_pass_m"))
        self.heading_kp = float(g("heading_kp"))
        self.pose = None          # (x, y, yaw) from the EKF: yaw from the gyro
        self.pose_s = -1e9
        self.create_subscription(Odometry, str(g("odom_topic")), self._on_odom, 10)
        self.escape = None   # (phase end time, phase, turn direction)
        self.tof = {"front": math.inf, "left": math.inf, "right": math.inf}
        self.last_v = self.last_w = 0.0
        self.dt = 1.0 / max(1.0, float(g("publish_rate_hz")))
        self.next_wander_s = self._now() + self._wander_gap()
        self.wander_until_s = 0.0
        self.wander_dir = 0.0

        self.last_scan = None
        self.last_scan_s = 0.0
        self.last_teleop_s = -1e9
        self.start_s = self._now()
        self.finished = False
        self.armed = None
        self.turn_dir = 0.0  # keep turning the same way until the way ahead is clear
        self.cmd_pub = self.create_publisher(Twist, str(g("cmd_vel_topic")), 10)
        self.arm_pub = self.create_publisher(Bool, str(g("arm_topic")), 10)
        self.create_subscription(LaserScan, str(g("scan_topic")), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, str(g("teleop_topic")), self._on_teleop, 10)
        self.create_subscription(Bool, str(g("enable_topic")), self._on_enable, 10)
        for topic in g("tof_topics"):
            key = topic.rstrip("/").split("/")[-1]
            self.create_subscription(Range, topic, lambda m, k=key: self.tof.__setitem__(k, m.range),
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / max(1.0, float(g("publish_rate_hz"))), self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _wander_gap(self) -> float:
        return random.uniform(*self.wander) if self.wander[1] > 0 else math.inf

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan, self.last_scan_s = msg, self._now()

    def _on_odom(self, m: Odometry) -> None:
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose, self.pose_s = (m.pose.pose.position.x, m.pose.pose.position.y, yaw), self._now()

    def _on_teleop(self, msg: Twist) -> None:
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_teleop_s = self._now()

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = msg.data

    def _arm(self, state: bool) -> None:
        self.arm_pub.publish(Bool(data=state))
        if state != self.armed:
            self.get_logger().info("exploring" if state else "paused (disarmed)")
        self.armed = state

    def _tick(self) -> None:
        now = self._now()
        if self.max_run_s > 0 and now - self.start_s > self.max_run_s:
            if not self.finished:
                self.get_logger().info(f"max_run_s {self.max_run_s:.0f} s reached; stopping. "
                                       f"ToF guard triggers: {self.tof_events}")
                self.finished = True
            self.cmd_pub.publish(Twist())
            self._arm(False)
            return
        if now - self.last_teleop_s < self.override_hold_s:
            self._arm(False)          # publish nothing: the gamepad takes over through the mux
            return
        if not self.enabled or self.last_scan is None or now - self.last_scan_s > self.scan_timeout_s:
            self.cmd_pub.publish(Twist())
            self._arm(False)
            return
        cmd = self._wall_follow(self.last_scan) if self.mode == "wall_follow" else self._command(self.last_scan)
        cmd = self._tof_guard(cmd)
        self.cmd_pub.publish(self._smooth(cmd))
        self._arm(True)

    def _tof_guard(self, cmd: Twist) -> Twist:
        """Close-range avoidance from the VL6180X (they see low objects below the lidar plane)."""
        front, left, right = self.tof["front"], self.tof["left"], self.tof["right"]
        hit = {"front": front < self.tof_stop, "left": left < self.tof_side, "right": right < self.tof_side}
        for k, on in hit.items():
            if on and not self.tof_active[k]:
                self.tof_events[k] += 1
                self.get_logger().info(f"ToF {k} {self.tof[k]:.3f} m: "
                                       + ("stop and turn" if k == "front" else "steer away"))
            self.tof_active[k] = on
        if hit["left"]:
            cmd.angular.z = min(cmd.angular.z, 0.0) - self.tof_steer      # turn right, away from it
            cmd.linear.x = min(cmd.linear.x, self.crawl_speed)
        if hit["right"]:
            cmd.angular.z = max(cmd.angular.z, 0.0) + self.tof_steer      # turn left
            cmd.linear.x = min(cmd.linear.x, self.crawl_speed)
        now = self._now()
        have_pose = self.pose is not None and now - self.pose_s < 0.5
        # a front hit starts the sidestep; while moving sideways or turning back it starts it again with the
        # same side and the same original heading (step further out); never during the first turn
        if hit["front"] and (self.escape is None or self.escape["phase"] in ("side", "return", "pass")):
            if self.escape is None:
                turn = -self.side if self.mode == "wall_follow" else (1.0 if left >= right else -1.0)
                yaw0 = self.pose[2] if have_pose else None
            else:
                turn, yaw0 = self.escape["turn"], self.escape["yaw0"]
            self.escape = {"phase": "back", "end": now + self.esc_back_s, "turn": turn, "yaw0": yaw0}
        if self.escape is None:
            return cmd
        e = self.escape
        wrap = lambda a: math.atan2(math.sin(a), math.cos(a))  # noqa: E731
        if e["phase"] == "back" and now >= e["end"]:
            e.update(phase="turn", end=now + self.esc_turn_s)
            if have_pose and e["yaw0"] is None:
                e["yaw0"] = self.pose[2]
        if e["phase"] == "turn":
            done = (abs(wrap(self.pose[2] - e["yaw0"])) >= self.esc_turn if have_pose and e["yaw0"] is not None
                    else now >= e["end"])
            if done and not hit["front"]:
                e.update(phase="side", end=now + self.esc_straight_s,
                         xy=self.pose[:2] if have_pose else None)
        if e["phase"] == "side":
            done = (math.hypot(self.pose[0] - e["xy"][0], self.pose[1] - e["xy"][1]) >= self.esc_side
                    if have_pose and e["xy"] is not None else now >= e["end"])
            if done:
                if have_pose and e["yaw0"] is not None:
                    e.update(phase="return")
                else:
                    self.escape = None
                    return cmd
        if e["phase"] == "return":
            # back to the heading before the turn (yaw0 minus the turn), then hand over to the mode
            target = wrap(e["yaw0"])
            if not have_pose:
                self.escape = None
                return cmd
            if abs(wrap(self.pose[2] - target)) < math.radians(8):
                e.update(phase="pass", xy=self.pose[:2], seen=False, clear_since=None)
        if e["phase"] == "pass":
            side_key = "right" if e["turn"] > 0 else "left"     # the object is on the side we stepped from
            near = self.tof[side_key] < 0.20
            if near:
                e["seen"], e["clear_since"] = True, None
            elif e["seen"] and e["clear_since"] is None:
                e["clear_since"] = now
            gone = e["seen"] and e["clear_since"] is not None and now - e["clear_since"] > 0.3
            far = math.hypot(self.pose[0] - e["xy"][0], self.pose[1] - e["xy"][1]) >= self.esc_pass
            if gone or far:
                self.get_logger().info("passed the object" if gone else "pass distance reached")
                self.escape = None
                return cmd
        out = Twist()
        if e["phase"] == "back":
            out.linear.x = -self.esc_back_v
        elif e["phase"] == "turn":
            out.angular.z = e["turn"] * 0.7 * self.turn_speed
        elif e["phase"] in ("side", "pass"):
            # straight line on the gyro heading: sideways leg at yaw0 + turn x escape angle, pass leg at yaw0
            out.linear.x = self.crawl_speed if e["phase"] == "side" else self.linear_speed
            if have_pose and e["yaw0"] is not None:
                target = e["yaw0"] + (e["turn"] * self.esc_turn if e["phase"] == "side" else 0.0)
                out.angular.z = max(-0.3, min(0.3, self.heading_kp * wrap(target - self.pose[2])))
            side_key = "right" if e["turn"] > 0 else "left"
            if e["phase"] == "pass" and self.tof[side_key] < 0.04:
                out.angular.z += e["turn"] * self.tof_steer     # only if it is about to touch
        else:
            out.angular.z = -e["turn"] * 0.7 * self.turn_speed
        return out
        return cmd

    def _smooth(self, cmd: Twist) -> Twist:
        """Rate-limit the command: speed ramps up gently but drops at once (braking is never delayed);
        the turn rate changes by at most angular_accel_radps2, so no step turns that make the wheels slip."""
        v, w = cmd.linear.x, cmd.angular.z
        self.last_v = min(v, self.last_v + self.v_accel * self.dt) if v > self.last_v else v
        self.last_w += max(-self.w_accel * self.dt, min(self.w_accel * self.dt, w - self.last_w))
        out = Twist()
        out.linear.x, out.angular.z = self.last_v, self.last_w
        return out

    def _points(self, scan: LaserScan):
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and self.min_range <= r <= scan.range_max:
                a = math.atan2(math.sin(scan.angle_min + i * scan.angle_increment + self.yaw),
                               math.cos(scan.angle_min + i * scan.angle_increment + self.yaw))
                yield a, r, r * math.cos(a), r * math.sin(a)

    def _command(self, scan: LaserScan) -> Twist:
        nearest_front = ahead = math.inf
        left = right = scan.range_max
        for a, r, x, y in self._points(scan):
            if x > 0:
                nearest_front = min(nearest_front, r)
                if abs(y) < self.half_width:
                    ahead = min(ahead, x)
            if self.half_width < y and 0 < a <= self.side_angle:
                left = min(left, r)
            elif y < -self.half_width and -self.side_angle <= a < 0:
                right = min(right, r)
        cmd = Twist()
        now = self._now()
        if now >= self.next_wander_s and self.wander_until_s < now:
            # start a random turn: duration = random angle / turn speed
            self.wander_dir = random.choice((-1.0, 1.0))
            self.wander_until_s = now + math.radians(random.uniform(30, 150)) / self.turn_speed
            self.next_wander_s = self.wander_until_s + self._wander_gap()
        if nearest_front < self.emergency_distance or ahead < self.obstacle_distance:
            if self.turn_dir == 0.0:
                self.turn_dir = 1.0 if left >= right else -1.0
                if random.random() < self.random_escape:
                    self.turn_dir = -self.turn_dir
            cmd.angular.z = self.turn_dir * self.turn_speed
            if nearest_front >= self.emergency_distance:
                cmd.linear.x = self.crawl_speed
        elif now < self.wander_until_s:
            cmd.angular.z = self.wander_dir * self.turn_speed   # random turn on the spot
        else:
            self.turn_dir = 0.0
            cmd.linear.x = self.linear_speed
            # steer off a near side wall only: sides beyond 1 m count as 1 m
            cmd.angular.z = max(-0.25, min(0.25, 0.35 * (min(left, 1.0) - min(right, 1.0))))
        return cmd

    def _wall_follow(self, scan: LaserScan) -> Twist:
        """Keep a wall on one side; arc toward the freest direction when something is ahead."""
        nearest_front = ahead = math.inf
        free = {1.0: 3.0, -1.0: 3.0}       # nearest return in the left / right half ahead (capped)
        wall = []
        for a, r, x, y in self._points(scan):
            if x > 0:
                nearest_front = min(nearest_front, r)
                if abs(y) < self.half_width:
                    ahead = min(ahead, x)
            if 0.25 < abs(a) < 1.4 and x > 0:
                s = 1.0 if y > 0 else -1.0
                free[s] = min(free[s], r)
            if -0.3 < x < 0.7 and 0.05 < self.side * y < 1.2:
                wall.append((x, y))
        # ToF: a near reading closes that direction (they also see low objects the lidar misses)
        if self.tof["left"] < self.tof_block:
            free[1.0] = 0.0
        if self.tof["right"] < self.tof_block:
            free[-1.0] = 0.0
        nearest_front = min(nearest_front, self.tof["front"] + self.tof_front_ahead)  # in the lidar's terms
        cmd = Twist()
        if nearest_front < self.emergency_distance:
            # last resort: slow spin toward the freer side
            if self.turn_dir == 0.0:
                self.turn_dir = 1.0 if free[1.0] >= free[-1.0] else -1.0
            cmd.angular.z = self.turn_dir * 0.5 * self.max_w
            return cmd
        if ahead < self.obstacle_distance:
            # arc (keep moving) toward the freer side; sharper the closer the obstacle
            if self.turn_dir == 0.0:
                self.turn_dir = 1.0 if free[1.0] >= free[-1.0] else -1.0
            k = 1.0 - (ahead - self.emergency_distance) / (self.obstacle_distance - self.emergency_distance)
            cmd.linear.x = self.crawl_speed
            cmd.angular.z = self.turn_dir * self.max_w * (0.5 + 0.5 * max(0.0, min(1.0, k)))
            return cmd
        self.turn_dir = 0.0
        cmd.linear.x = self.linear_speed
        if len(wall) >= 8:
            # least-squares line y = c + b x through the wall points: distance and angle to the wall
            n = len(wall)
            mx = sum(p[0] for p in wall) / n
            my = sum(p[1] for p in wall) / n
            sxx = sum((p[0] - mx) ** 2 for p in wall)
            b = sum((p[0] - mx) * (p[1] - my) for p in wall) / sxx if sxx > 1e-6 else 0.0
            c = my - b * mx
            dist = abs(c) / math.sqrt(1 + b * b)
            w = self.kp_a * math.atan(b) + self.side * self.kp_d * (dist - self.wall_distance)
        else:
            w = self.side * 0.35 * self.max_w   # no wall: curve gently toward the follow side to find one
        w += self.weave_amp * math.sin(2 * math.pi * self._now() / self.weave_period)
        cmd.angular.z = max(-self.max_w, min(self.max_w, w))
        return cmd

    def destroy_node(self) -> bool:
        self.cmd_pub.publish(Twist())
        self._arm(False)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RoomMappingExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
