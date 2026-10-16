"""Stationary IMU calibration helper.

Algorithm:
1. Subscribe to raw `sensor_msgs/msg/Imu` samples.
2. Assume the robot is standing still during the full sample window.
3. Average gyroscope readings to estimate static gyro bias in rad/s.
4. Average accelerometer readings to record the observed gravity vector.
5. Compare the acceleration norm with standard gravity as a sanity check.
6. Write a YAML file that later filtering/localization nodes can consume.
"""

# References (full entries: docs/references.md at the repository root):
#   [R42] Siegwart et al., Autonomous Mobile Robots: gyroscope bias and gravity reference
#   [R12] REP 103, units and axes: IMU axes
# More details: README.md of this package

from pathlib import Path
from statistics import fmean
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


Vector3 = Tuple[float, float, float]


class ImuCalibrationNode(Node):
    """Collect stationary IMU samples and save bias/gravity estimates."""

    def __init__(self) -> None:
        super().__init__("imu_calibration_node")

        # `sample_count` controls calibration duration at the IMU topic rate.
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("sample_count", 500)
        self.declare_parameter(
            "output_path", "~/.ros/learn2race/imu_calibration.yaml"
        )
        self.declare_parameter("expected_gravity_mps2", 9.80665)
        self.declare_parameter("max_gravity_error_mps2", 1.5)

        self.sample_count = int(self.get_parameter("sample_count").value)
        self.output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        self.expected_gravity = float(self.get_parameter("expected_gravity_mps2").value)
        self.max_gravity_error = float(
            self.get_parameter("max_gravity_error_mps2").value
        )
        self.done = False

        # Store only primitive tuples so the averaging code is independent of ROS msgs.
        self.gyro_samples: List[Vector3] = []
        self.accel_samples: List[Vector3] = []

        imu_topic = str(self.get_parameter("imu_topic").value)
        self.create_subscription(Imu, imu_topic, self._imu_callback, 50)
        self.get_logger().info(
            f"Keep the robot completely still. Collecting "
            f"{self.sample_count} samples from {imu_topic}."
        )

    def _imu_callback(self, msg: Imu) -> None:
        if self.done:
            return

        self.gyro_samples.append(
            (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        )
        self.accel_samples.append(
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        )

        count = len(self.gyro_samples)
        if count % 100 == 0:
            self.get_logger().info(f"IMU calibration samples: {count}/{self.sample_count}")

        if count >= self.sample_count:
            self.done = True
            self._write_calibration()
            rclpy.try_shutdown()

    def _write_calibration(self) -> None:
        gyro_bias = self._mean_vector(self.gyro_samples)
        accel_mean = self._mean_vector(self.accel_samples)
        accel_norm = self._norm(accel_mean)
        gravity_error = abs(accel_norm - self.expected_gravity)

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(
            "\n".join(
                [
                    "# Learn2Race stationary IMU calibration.",
                    "# The robot must be still for these offsets to be valid.",
                    "imu_calibration:",
                    f"  sample_count: {len(self.gyro_samples)}",
                    f"  expected_gravity_mps2: {self.expected_gravity:.6f}",
                    "  gyro_bias_radps:",
                    f"    x: {gyro_bias[0]:.9f}",
                    f"    y: {gyro_bias[1]:.9f}",
                    f"    z: {gyro_bias[2]:.9f}",
                    "  observed_accel_mps2:",
                    f"    x: {accel_mean[0]:.9f}",
                    f"    y: {accel_mean[1]:.9f}",
                    f"    z: {accel_mean[2]:.9f}",
                    f"  observed_accel_norm_mps2: {accel_norm:.9f}",
                    f"  gravity_error_mps2: {gravity_error:.9f}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

        if gravity_error > self.max_gravity_error:
            self.get_logger().warn(
                "Observed acceleration norm differs from gravity by "
                f"{gravity_error:.3f} m/s^2. Check that the robot was still and level."
            )

        self.get_logger().info(f"IMU calibration saved to {self.output_path}")

    @staticmethod
    def _mean_vector(samples: List[Vector3]) -> Vector3:
        return (
            fmean(sample[0] for sample in samples),
            fmean(sample[1] for sample in samples),
            fmean(sample[2] for sample in samples),
        )

    @staticmethod
    def _norm(vector: Vector3) -> float:
        return (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** 0.5


def main() -> None:
    rclpy.init()
    node = ImuCalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
