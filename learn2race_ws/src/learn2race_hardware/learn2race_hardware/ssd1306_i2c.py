"""Small SSD1306 I2C OLED driver for monochrome page buffers."""

# References (full entries: docs/references.md at the repository root):
#   [R78] SSD1306 datasheet: SSD1306 commands
# More details: README.md of this package

try:
    from smbus2 import SMBus
except ImportError:  # Hardware library is only expected on the Raspberry Pi.
    SMBus = None


class Ssd1306I2c:
    """Initialize the display and send page-formatted framebuffer bytes."""

    DEFAULT_ADDRESS = 0x3C

    def __init__(self, bus_id: int, address: int, width: int, height: int) -> None:
        if SMBus is None:
            raise RuntimeError("Python package smbus2 is not installed")
        if height % 8 != 0:
            raise ValueError(f"SSD1306 height must be divisible by 8, got {height}")

        self.bus = SMBus(bus_id)
        self.address = address
        self.width = width
        self.height = height
        self.pages = height // 8
        try:
            self._init_display()
        except Exception:
            self.bus.close()
            raise

    def close(self) -> None:
        try:
            self.clear()
        except OSError:
            pass  # Unplugged OLED must not prevent closing the I2C descriptor.
        finally:
            self.bus.close()

    def clear(self) -> None:
        self.show(bytes(self.width * self.pages))

    def show(self, framebuffer: bytes) -> None:
        expected_size = self.width * self.pages
        if len(framebuffer) != expected_size:
            raise ValueError(
                f"SSD1306 framebuffer must be {expected_size} bytes, got {len(framebuffer)}"
            )

        # Set column/page address windows before streaming display RAM.
        self._command(0x21)
        self._command(0)
        self._command(self.width - 1)
        self._command(0x22)
        self._command(0)
        self._command(self.pages - 1)

        for offset in range(0, len(framebuffer), 16):
            self.bus.write_i2c_block_data(
                self.address, 0x40, list(framebuffer[offset : offset + 16])
            )

    def image_to_pages(self, image) -> bytes:
        """Convert a 1-bit Pillow image to SSD1306 vertical page byte order."""

        pixels = image.load()
        output = bytearray(self.width * self.pages)
        index = 0
        for page in range(self.pages):
            for x in range(self.width):
                value = 0
                for bit in range(8):
                    y = page * 8 + bit
                    if pixels[x, y]:
                        value |= 1 << bit
                output[index] = value
                index += 1
        return bytes(output)

    def _init_display(self) -> None:
        # Initialization sequence follows the common SSD1306 page-addressed
        # mode for 128x32 and 128x64 monochrome OLED modules.
        sequence = [
            0xAE,
            0xD5,
            0x80,
            0xA8,
            self.height - 1,
            0xD3,
            0x00,
            0x40,
            0x8D,
            0x14,
            0x20,
            0x00,
            0xA1,
            0xC8,
            0xDA,
            0x02 if self.height == 32 else 0x12,
            0x81,
            0xCF,
            0xD9,
            0xF1,
            0xDB,
            0x40,
            0xA4,
            0xA6,
            0x2E,
            0xAF,
        ]
        for command in sequence:
            self._command(command)
        self.clear()

    def _command(self, command: int) -> None:
        self.bus.write_byte_data(self.address, 0x00, command)
