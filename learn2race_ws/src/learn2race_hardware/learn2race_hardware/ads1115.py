"""ADS1115 16-bit I2C ADC driver with single-ended and differential modes.

The differential mode is what makes current sensing possible with this part: a
shunt drop of a few tens of millivolts is unusable on a single-ended input
referenced to ground, but at the +/-0.256 V programmable gain setting the
device resolves 7.8 uV per bit, which is roughly 1.6 mA across a 5 mOhm shunt.

Note the hardware constraint that shapes the wiring. Inputs must stay within
GND - 0.3 V to VDD + 0.3 V, so a high-side shunt on a 4S pack is impossible:
the common-mode voltage would be 16.8 V against a 3.3 V supply. The shunt must
sit in the ground return.
"""

# References (full entries: docs/references.md at the repository root):
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS1115 registers, PGA
# More details: README.md of this package

REG_CONVERSION = 0x00
REG_CONFIG = 0x01

# Config register MUX field. Single-ended channels and the two differential
# pairs the device supports; AIN0-AIN1 and AIN2-AIN3 are the usable pairs.
MUX = {
    0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000,        # single-ended AIN0..AIN3
    (0, 1): 0x0000,                                     # differential AIN0-AIN1
    (0, 3): 0x1000,
    (1, 3): 0x2000,
    (2, 3): 0x3000,                                     # differential AIN2-AIN3
}

# Programmable gain amplifier: full-scale range in volts -> config bits.
PGA = {
    6.144: 0x0000,
    4.096: 0x0200,
    2.048: 0x0400,
    1.024: 0x0600,
    0.512: 0x0800,
    0.256: 0x0A00,
}

# Data rate. 128 SPS is the default; 8 SPS averages hardest and is the right
# choice for a pack reading taken once a second next to a switching motor.
DATA_RATE = {8: 0x0000, 16: 0x0020, 32: 0x0040, 64: 0x0060,
             128: 0x0080, 250: 0x00A0, 475: 0x00C0, 860: 0x00E0}

SINGLE_SHOT_START = 0x8000
MODE_SINGLE_SHOT = 0x0100
COMPARATOR_OFF = 0x0003


class Ads1115:

    def __init__(self, bus, address=0x48):
        self.bus = bus
        self.address = address

    def _convert(self, mux_key, full_scale, data_rate):
        if mux_key not in MUX:
            raise ValueError(f'unsupported input selection {mux_key}')
        if full_scale not in PGA:
            raise ValueError(f'unsupported full-scale range {full_scale}')

        config = (SINGLE_SHOT_START | MUX[mux_key] | PGA[full_scale] |
                  MODE_SINGLE_SHOT | DATA_RATE[data_rate] | COMPARATOR_OFF)

        self.bus.write_i2c_block_data(
            self.address, REG_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])

        # Wait for the conversion rather than sleeping a fixed interval: at
        # 8 SPS a fixed delay would either waste time or read a stale value.
        for _ in range(200):
            status = self.bus.read_i2c_block_data(self.address, REG_CONFIG, 2)
            if (status[0] & 0x80) != 0:          # OS bit set: conversion done
                break

        data = self.bus.read_i2c_block_data(self.address, REG_CONVERSION, 2)
        raw = (data[0] << 8) | data[1]
        return raw - 65536 if raw > 32767 else raw

    def read_channel(self, channel, full_scale=4.096, data_rate=128):
        """Single-ended reading, in raw counts."""
        return self._convert(channel, full_scale, data_rate)

    def read_volts(self, channel, full_scale=4.096, data_rate=128):
        raw = self.read_channel(channel, full_scale, data_rate)
        return (raw / 32767.0) * full_scale

    def read_differential(self, pair=(2, 3), full_scale=0.256, data_rate=8):
        """Differential reading across a pair, in volts.

        Defaults suit a low-side shunt: the narrowest gain for resolution, and
        the slowest data rate because the internal averaging is the cheapest
        rejection of motor switching noise available.
        """
        raw = self._convert(tuple(pair), full_scale, data_rate)
        return (raw / 32767.0) * full_scale
