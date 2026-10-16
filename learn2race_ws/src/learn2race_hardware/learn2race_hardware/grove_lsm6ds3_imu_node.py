"""ROS adapter for the Grove 6-Axis Accelerometer & Gyroscope v1.0.

The hardware driver in `lsm6ds3.py` handles I2C registers. This node converts
samples to ROS `Imu` and `Temperature` messages and owns topic/frame parameters.

It also applies the axis mapping and the gyroscope bias, because the sensor
reports its own axes and ROS requires a body frame with x forward, y left and
z up (REP-103). How the sensor axes relate to the robot depends on how the
breakout is mounted, so the mapping is a per-robot parameter determined with
`ros2 run learn2race_evaluation calibrate_imu_axes` rather than a constant.

A wrong mapping does not fail loudly. The estimator fuses one axis as another,
produces a plausible pose, and the robot drifts in a way that resembles poor
wheel calibration, so the fault is easily attributed to the wrong subsystem.
"""

# References (full entries: docs/references.md at the repository root):
#   [R71] LSM6DS3 datasheet: LSM6DS3
#   [R74] Seeed Grove module wiki: Grove 6-axis module
#   [R12] REP 103, units and axes: IMU axes and units
# More details: README.md of this package; lsm6ds3.py

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

from .lsm6ds3 import Lsm6ds3


class GroveLsm6ds3ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("grove_lsm6ds3_imu_node")

        # I2C parameters identify the bus and the LSM6DS3 address on that bus.
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 106)
        # Topic/frame parameters define the ROS interface exposed to the robot.
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("temperature_topic", "/imu/temperature")
        self.declare_parameter("publish_temperature", True)
        # Sensor ranges set the LSM6DS3 register configuration and output scale.
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("sensor_odr_hz", 104.0)
        self.declare_parameter("accel_range_g", 2)
        self.declare_parameter("gyro_range_dps", 245)
        # Variances are optional hints for downstream filters.
        self.declare_parameter("angular_velocity_variance", 0.0)
        self.declare_parameter("linear_acceleration_variance", 0.0)
        self.declare_parameter("temperature_variance", 0.0)
        # Axis mapping into the REP-103 body frame. Source indices are 0, 1
        # and 2 for the sensor X, Y and Z channels; signs are +1 or -1. The
        # defaults are the identity mapping, which holds only if the breakout
        # happens to be mounted with its axes aligned to the robot.
        self.declare_parameter("axis_x_source", 0)
        self.declare_parameter("axis_x_sign", 1)
        self.declare_parameter("axis_y_source", 1)
        self.declare_parameter("axis_y_sign", 1)
        self.declare_parameter("axis_z_source", 2)
        self.declare_parameter("axis_z_sign", 1)
        self.declare_parameter("gyro_sign", 1)
        # Set true once calibrate_imu_axes has confirmed the mapping, so a
        # correctly mounted breakout no longer warns about the identity.
        self.declare_parameter("axis_mapping_verified", False)
        # Gyroscope bias in rad/s, subtracted after remapping. Measured with
        # `calibrate_imu`; an uncorrected bias integrates into heading error.
        self.declare_parameter("gyro_bias", [0.0, 0.0, 0.0])
        self.declare_parameter("dry_run", False)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.angular_velocity_variance = float(
            self.get_parameter("angular_velocity_variance").value
        )
        self.linear_acceleration_variance = float(
            self.get_parameter("linear_acceleration_variance").value
        )
        self.temperature_variance = float(self.get_parameter("temperature_variance").value)
        self.last_error_log_time = self.get_clock().now()

        self.axis_sources = (int(self.get_parameter("axis_x_source").value),
                             int(self.get_parameter("axis_y_source").value),
                             int(self.get_parameter("axis_z_source").value))
        self.axis_signs = (float(self.get_parameter("axis_x_sign").value),
                           float(self.get_parameter("axis_y_sign").value),
                           float(self.get_parameter("axis_z_sign").value))
        self.gyro_sign = float(self.get_parameter("gyro_sign").value)
        self.gyro_bias = [float(v) for v in self.get_parameter("gyro_bias").value]

        if sorted(self.axis_sources) != [0, 1, 2]:
            raise ValueError(
                "axis sources must be a permutation of 0, 1 and 2; "
                f"got {self.axis_sources}. Re-run calibrate_imu_axes."
            )
        if (self.axis_sources == (0, 1, 2) and self.axis_signs == (1.0, 1.0, 1.0)
                and not bool(self.get_parameter("axis_mapping_verified").value)):
            self.get_logger().warn(
                "IMU axis mapping is the identity. If the breakout is not "
                "mounted aligned with the robot, orientation estimates will be "
                "wrong in a way that looks like poor odometry calibration. Run "
                "calibrate_imu_axes to confirm."
            )

        self.imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), 10
        )
        self.temperature_pub = None
        if bool(self.get_parameter("publish_temperature").value):
            self.temperature_pub = self.create_publisher(
                Temperature, str(self.get_parameter("temperature_topic").value), 10
            )

        self.imu: Optional[Lsm6ds3] = None
        self._init_imu()

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self._publish_sample)

    def _init_imu(self) -> None:
        if self.dry_run:
            self.get_logger().warn("LSM6DS3 IMU dry_run=true; not touching I2C.")
            return

        bus = int(self.get_parameter("i2c_bus").value)
        address = self._parse_address(self.get_parameter("i2c_address").value)

        try:
            self.imu = Lsm6ds3(
                bus_id=bus,
                address=address,
                odr_hz=float(self.get_parameter("sensor_odr_hz").value),
                accel_range_g=int(self.get_parameter("accel_range_g").value),
                gyro_range_dps=int(self.get_parameter("gyro_range_dps").value),
            )
            who_am_i = self.imu.begin()
            self.get_logger().info(
                f"Started Grove LSM6DS3 IMU on bus {bus}, address 0x{address:02X}, "
                f"WHO_AM_I=0x{who_am_i:02X}, frame={self.frame_id}."
            )
        except (OSError, RuntimeError, ValueError) as exc:
            self.imu = None
            self.dry_run = True
            self.get_logger().error(
                "Could not start Grove LSM6DS3 IMU. Check wiring, I2C enablement, "
                f"and address with `i2cdetect -y {bus}`. Falling back to dry-run. "
                f"Error: {exc}"
            )

    def _remap(self, values, extra_sign: float):
        """Reorder and re-sign sensor channels into the REP-103 body frame."""
        return [extra_sign * self.axis_signs[i] * values[self.axis_sources[i]]
                for i in range(3)]

    def _publish_sample(self) -> None:
        if self.dry_run or self.imu is None:
            return

        try:
            sample = self.imu.read_sample()
        except OSError as exc:
            self._log_read_error(exc)
            return

        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id
        # This node publishes raw accel/gyro only; orientation is intentionally unknown.
        imu_msg.orientation_covariance[0] = -1.0

        gyro = self._remap(sample.angular_velocity_radps, self.gyro_sign)
        imu_msg.angular_velocity.x = gyro[0] - self.gyro_bias[0]
        imu_msg.angular_velocity.y = gyro[1] - self.gyro_bias[1]
        imu_msg.angular_velocity.z = gyro[2] - self.gyro_bias[2]
        imu_msg.angular_velocity_covariance[0] = self.angular_velocity_variance
        imu_msg.angular_velocity_covariance[4] = self.angular_velocity_variance
        imu_msg.angular_velocity_covariance[8] = self.angular_velocity_variance

        accel = self._remap(sample.linear_acceleration_mps2, 1.0)
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance[0] = self.linear_acceleration_variance
        imu_msg.linear_acceleration_covariance[4] = self.linear_acceleration_variance
        imu_msg.linear_acceleration_covariance[8] = self.linear_acceleration_variance
        self.imu_pub.publish(imu_msg)

        if self.temperature_pub is not None:
            temperature_msg = Temperature()
            temperature_msg.header.stamp = stamp
            temperature_msg.header.frame_id = self.frame_id
            temperature_msg.temperature = sample.temperature_c
            temperature_msg.variance = self.temperature_variance
            self.temperature_pub.publish(temperature_msg)

    def _log_read_error(self, exc: OSError) -> None:
        now = self.get_clock().now()
        elapsed_s = (now - self.last_error_log_time).nanoseconds / 1e9
        if elapsed_s >= 2.0:
            self.last_error_log_time = now
            self.get_logger().warn(f"Could not read LSM6DS3 sample: {exc}")

    @staticmethod
    def _parse_address(value) -> int:
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def destroy_node(self) -> bool:
        if self.imu is not None:
            self.imu.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GroveLsm6ds3ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
