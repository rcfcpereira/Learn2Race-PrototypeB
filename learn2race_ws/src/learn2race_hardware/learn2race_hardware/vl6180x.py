"""Minimal VL6180X time-of-flight range sensor driver."""

# References (full entries: docs/references.md at the repository root):
#   [R72] VL6180X datasheet, TOF050C module: VL6180X registers
# More details: README.md of this package

import time
from dataclasses import dataclass

try:
    from smbus2 import i2c_msg
except ImportError:  # Hardware library is only expected on the Raspberry Pi.
    i2c_msg = None


@dataclass(frozen=True)
class Vl6180xRange:
    distance_mm: int
    status: int


class Vl6180x:
    """Initialize a VL6180X and read single-shot distance measurements."""

    DEFAULT_ADDRESS = 0x29

    IDENTIFICATION_MODEL_ID = 0x000
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x014
    SYSTEM_INTERRUPT_CLEAR = 0x015
    SYSTEM_FRESH_OUT_OF_RESET = 0x016
    SYSRANGE_START = 0x018
    SYSRANGE_MAX_CONVERGENCE_TIME = 0x01C
    SYSRANGE_VHV_RECALIBRATE = 0x02E
    SYSRANGE_VHV_REPEAT_RATE = 0x031
    RESULT_INTERRUPT_STATUS_GPIO = 0x04F
    RESULT_RANGE_STATUS = 0x04D
    RESULT_RANGE_VAL = 0x062

    MODEL_ID = 0xB4

    def __init__(self, bus, address: int = DEFAULT_ADDRESS) -> None:
        if i2c_msg is None:
            raise RuntimeError("Python package smbus2 is not installed")
        self.bus = bus
        self.address = address

    def begin(self) -> int:
        model_id = self._read_u8(self.IDENTIFICATION_MODEL_ID)
        if model_id != self.MODEL_ID:
            raise RuntimeError(
                f"Unexpected VL6180X model id 0x{model_id:02X}; expected 0xB4"
            )

        self._apply_private_settings()
        self._apply_recommended_settings()
        self._write_u8(self.SYSTEM_FRESH_OUT_OF_RESET, 0x00)
        self.clear_interrupts()
        return model_id

    def read_range(self, timeout_s: float = 0.05) -> Vl6180xRange:
        self.clear_interrupts()
        self._write_u8(self.SYSRANGE_START, 0x01)

        # Poll the range-ready interrupt bit until the single-shot read completes.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._read_u8(self.RESULT_INTERRUPT_STATUS_GPIO) & 0x04:
                status = self.range_status()
                distance_mm = self._read_u8(self.RESULT_RANGE_VAL)
                self.clear_interrupts()
                return Vl6180xRange(distance_mm=distance_mm, status=status)
            time.sleep(0.001)

        raise TimeoutError(f"VL6180X range read timed out after {timeout_s:.3f}s")

    def range_status(self) -> int:
        return (self._read_u8(self.RESULT_RANGE_STATUS) >> 4) & 0x0F

    def clear_interrupts(self) -> None:
        self._write_u8(self.SYSTEM_INTERRUPT_CLEAR, 0x07)

    def _apply_private_settings(self) -> None:
        for register, value in (
            (0x0207, 0x01),
            (0x0208, 0x01),
            (0x0096, 0x00),
            (0x0097, 0xFD),
            (0x00E3, 0x00),
            (0x00E4, 0x04),
            (0x00E5, 0x02),
            (0x00E6, 0x01),
            (0x00E7, 0x03),
            (0x00F5, 0x02),
            (0x00D9, 0x05),
            (0x00DB, 0xCE),
            (0x00DC, 0x03),
            (0x00DD, 0xF8),
            (0x009F, 0x00),
            (0x00A3, 0x3C),
            (0x00B7, 0x00),
            (0x00BB, 0x3C),
            (0x00B2, 0x09),
            (0x00CA, 0x09),
            (0x0198, 0x01),
            (0x01B0, 0x17),
            (0x01AD, 0x00),
            (0x00FF, 0x05),
            (0x0100, 0x05),
            (0x0199, 0x05),
            (0x01A6, 0x1B),
            (0x01AC, 0x3E),
            (0x01A7, 0x1F),
            (0x0030, 0x00),
        ):
            self._write_u8(register, value)

    def _apply_recommended_settings(self) -> None:
        for register, value in (
            (0x0011, 0x10),
            (0x010A, 0x30),
            (0x003F, 0x46),
            (0x0031, 0xFF),
            (0x0040, 0x63),
            (0x002E, 0x01),
            (self.SYSRANGE_MAX_CONVERGENCE_TIME, 0x09),
            (self.SYSRANGE_VHV_REPEAT_RATE, 0x31),
            (self.SYSRANGE_VHV_RECALIBRATE, 0x01),
            (self.SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24),
        ):
            self._write_u8(register, value)

    def _read_u8(self, register: int) -> int:
        # VL6180X registers are 16-bit addresses, so smbus2 i2c_msg is clearer.
        write = i2c_msg.write(
            self.address, [(register >> 8) & 0xFF, register & 0xFF]
        )
        read = i2c_msg.read(self.address, 1)
        self.bus.i2c_rdwr(write, read)
        return list(read)[0]

    def _write_u8(self, register: int, value: int) -> None:
        self.bus.write_i2c_block_data(
            self.address,
            (register >> 8) & 0xFF,
            [register & 0xFF, value & 0xFF],
        )
