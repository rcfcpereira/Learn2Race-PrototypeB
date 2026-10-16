"""Minimal LSM6DS3 I2C driver used by the Grove 6-Axis IMU node."""

# References (full entries: docs/references.md at the repository root):
#   [R71] LSM6DS3 datasheet: LSM6DS3 registers
# More details: README.md of this package

import math
import time
from dataclasses import dataclass
from typing import Tuple

try:
    from smbus2 import SMBus
except ImportError:  # Hardware library is only expected on the Raspberry Pi.
    SMBus = None


@dataclass(frozen=True)
class Lsm6ds3Sample:
    linear_acceleration_mps2: Tuple[float, float, float]
    angular_velocity_radps: Tuple[float, float, float]
    temperature_c: float


class Lsm6ds3:
    """Configure and read accelerometer, gyroscope, and temperature samples."""

    WHO_AM_I = 0x0F
    # LSM6DS3 reports 0x69; the LSM6DS3TR-C fitted on newer Grove boards reports
    # 0x6A. Both share the registers and scales used here except temperature.
    _TEMP_LSB_PER_C = {0x69: 16.0, 0x6A: 256.0}
    CTRL1_XL = 0x10
    CTRL2_G = 0x11
    CTRL3_C = 0x12
    OUT_TEMP_L = 0x20

    _ODR_BITS = {
        12.5: 0x10,
        26.0: 0x20,
        52.0: 0x30,
        104.0: 0x40,
        208.0: 0x50,
        416.0: 0x60,
        833.0: 0x70,
        1660.0: 0x80,
    }
    _ACCEL_BITS = {
        2: 0x00,
        16: 0x04,
        4: 0x08,
        8: 0x0C,
    }
    _ACCEL_MG_PER_LSB = {
        2: 0.061,
        4: 0.122,
        8: 0.244,
        16: 0.488,
    }
    _GYRO_BITS = {
        125: 0x02,
        245: 0x00,
        500: 0x04,
        1000: 0x08,
        2000: 0x0C,
    }
    _GYRO_MDPS_PER_LSB = {
        125: 4.375,
        245: 8.75,
        500: 17.5,
        1000: 35.0,
        2000: 70.0,
    }

    def __init__(
        self,
        bus_id: int,
        address: int = 0x6A,
        odr_hz: float = 104.0,
        accel_range_g: int = 2,
        gyro_range_dps: int = 245,
    ) -> None:
        if SMBus is None:
            raise RuntimeError("Python package smbus2 is not installed")
        if odr_hz not in self._ODR_BITS:
            raise ValueError(f"Unsupported LSM6DS3 ODR: {odr_hz}")
        if accel_range_g not in self._ACCEL_BITS:
            raise ValueError(f"Unsupported LSM6DS3 accel range: {accel_range_g}g")
        if gyro_range_dps not in self._GYRO_BITS:
            raise ValueError(f"Unsupported LSM6DS3 gyro range: {gyro_range_dps} dps")

        self.bus = SMBus(bus_id)
        self.address = address
        self.odr_hz = odr_hz
        self.accel_range_g = accel_range_g
        self.gyro_range_dps = gyro_range_dps
        self.accel_scale = self._ACCEL_MG_PER_LSB[accel_range_g] * 9.80665 / 1000.0
        self.gyro_scale = self._GYRO_MDPS_PER_LSB[gyro_range_dps] * math.pi / 180000.0
        self.temp_lsb_per_c = 16.0

    def begin(self) -> int:
        who_am_i = self.bus.read_byte_data(self.address, self.WHO_AM_I)
        if who_am_i not in self._TEMP_LSB_PER_C:
            raise RuntimeError(
                f"Unexpected LSM6DS3 WHO_AM_I 0x{who_am_i:02X}; expected 0x69 or 0x6A (TR-C)"
            )
        self.temp_lsb_per_c = self._TEMP_LSB_PER_C[who_am_i]

        # BDU avoids mixed high/low bytes; IF_INC enables one block read for all axes.
        self.bus.write_byte_data(self.address, self.CTRL3_C, 0x44)
        odr_bits = self._ODR_BITS[self.odr_hz]
        self.bus.write_byte_data(
            self.address, self.CTRL1_XL, odr_bits | self._ACCEL_BITS[self.accel_range_g]
        )
        self.bus.write_byte_data(
            self.address, self.CTRL2_G, odr_bits | self._GYRO_BITS[self.gyro_range_dps]
        )
        time.sleep(0.05)
        return who_am_i

    def read_sample(self) -> Lsm6ds3Sample:
        # LSM6DS3 lays out temp, gyro XYZ, then accel XYZ as little-endian int16.
        data = self.bus.read_i2c_block_data(self.address, self.OUT_TEMP_L, 14)
        temp_raw = self._i16(data[0], data[1])
        gyro_raw = (
            self._i16(data[2], data[3]),
            self._i16(data[4], data[5]),
            self._i16(data[6], data[7]),
        )
        accel_raw = (
            self._i16(data[8], data[9]),
            self._i16(data[10], data[11]),
            self._i16(data[12], data[13]),
        )

        return Lsm6ds3Sample(
            linear_acceleration_mps2=tuple(value * self.accel_scale for value in accel_raw),
            angular_velocity_radps=tuple(value * self.gyro_scale for value in gyro_raw),
            temperature_c=25.0 + temp_raw / self.temp_lsb_per_c,
        )

    def close(self) -> None:
        self.bus.close()

    @staticmethod
    def _i16(low: int, high: int) -> int:
        value = low | (high << 8)
        if value & 0x8000:
            value -= 0x10000
        return value
