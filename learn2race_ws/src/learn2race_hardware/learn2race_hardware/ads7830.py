"""Minimal ADS7830 8-channel I2C ADC driver.

The ADS7830 is a single-ended 8-bit ADC used here to digitise the reflective
line-sensor array. One command byte selects the channel and the power-down mode;
the device then returns one data byte.
"""

# References (full entries: docs/references.md at the repository root):
#   [R70] ADS1115 / ADS7830 / PCF8591 datasheets: ADS7830 command byte
# More details: README.md of this package

CMD_SINGLE_ENDED = 0x84          # SD=1, PD1=PD0=1 (internal reference off, ADC on)
CHANNEL_MAP = [0, 4, 1, 5, 2, 6, 3, 7]   # datasheet single-ended channel order


class Ads7830:

    def __init__(self, bus, address=0x48):
        self.bus = bus
        self.address = address

    def read_channel(self, channel: int) -> int:
        """Return the raw 8-bit conversion for a single-ended channel (0-7)."""
        if not 0 <= channel <= 7:
            raise ValueError('channel must be 0-7')
        command = CMD_SINGLE_ENDED | (CHANNEL_MAP[channel] << 4)
        return self.bus.read_byte_data(self.address, command)

    def read_all(self):
        return [self.read_channel(ch) for ch in range(8)]
