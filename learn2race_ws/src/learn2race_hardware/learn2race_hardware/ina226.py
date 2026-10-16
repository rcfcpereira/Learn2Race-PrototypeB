"""Minimal INA226 driver (I2C voltage, current and power monitor).

The INA226 measures the drop across an external shunt and the bus voltage, and
computes current and power in hardware once the calibration register is set.
Calibration depends on the shunt resistance actually fitted, so it is a
parameter rather than a constant: a wrong shunt value produces a current
reading that is wrong by a fixed factor and otherwise looks entirely plausible.
"""

# References (full entries: docs/references.md at the repository root):
#   [R69] INA219 / INA226 datasheets, INA219_WE: INA226 calibration register
# More details: README.md of this package

REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01
REG_BUS_VOLTAGE = 0x02
REG_POWER = 0x03
REG_CURRENT = 0x04
REG_CALIBRATION = 0x05

BUS_VOLTAGE_LSB = 1.25e-3      # volts per bit, fixed by the device
SHUNT_VOLTAGE_LSB = 2.5e-6     # volts per bit, fixed by the device


class Ina226:

    def __init__(self, bus, address=0x40, shunt_ohms=0.002,
                 max_expected_amps=20.0):
        self.bus = bus
        self.address = address
        self.shunt_ohms = shunt_ohms

        # Current LSB is chosen so the full expected range fits in 15 bits.
        self.current_lsb = max_expected_amps / 32768.0
        calibration = int(0.00512 / (self.current_lsb * shunt_ohms))
        self._write(REG_CALIBRATION, calibration)

        # Averaging of 16 samples with 1.1 ms conversions: slow enough to
        # reject motor switching noise, fast enough for a 1 Hz publisher.
        self._write(REG_CONFIG, 0x4527)

    def _write(self, register, value):
        self.bus.write_i2c_block_data(
            self.address, register, [(value >> 8) & 0xFF, value & 0xFF])

    def _read(self, register, signed=False):
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        value = (data[0] << 8) | data[1]
        if signed and value > 32767:
            value -= 65536
        return value

    def bus_voltage(self) -> float:
        return self._read(REG_BUS_VOLTAGE) * BUS_VOLTAGE_LSB

    def shunt_voltage(self) -> float:
        return self._read(REG_SHUNT_VOLTAGE, signed=True) * SHUNT_VOLTAGE_LSB

    def current(self) -> float:
        return self._read(REG_CURRENT, signed=True) * self.current_lsb

    def pack_voltage(self) -> float:
        """Voltage at the source, before the shunt drop."""
        return self.bus_voltage() + self.shunt_voltage()
