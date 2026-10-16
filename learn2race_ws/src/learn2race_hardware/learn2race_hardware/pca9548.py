"""PCA9548/TCA9548A I2C multiplexer helper."""

# References (full entries: docs/references.md at the repository root):
#   [R77] PCA9548A / TCA9548A datasheet: PCA9548A channel select
# More details: README.md of this package


class Pca9548:
    """Select one downstream I2C channel at a time."""

    def __init__(self, bus, address: int = 0x70) -> None:
        self.bus = bus
        self.address = address

    def select_channel(self, channel: int) -> None:
        if channel < 0 or channel > 7:
            raise ValueError(f"PCA9548 channel must be 0..7, got {channel}")
        # Each bit in the control byte enables one mux channel.
        self.bus.write_byte(self.address, 1 << channel)

    def disable_all(self) -> None:
        self.bus.write_byte(self.address, 0x00)
