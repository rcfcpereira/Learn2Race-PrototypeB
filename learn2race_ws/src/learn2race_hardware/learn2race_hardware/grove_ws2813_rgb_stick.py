"""Small hardware wrapper for a WS2813 Grove RGB Stick."""

# References (full entries: docs/references.md at the repository root):
#   [R75] WS2813 datasheet, Linux spidev, rpi_ws281x: WS2813 bit timing; SPI encoding via spidev; rpi_ws281x
#   [R65] Raspberry Pi 4/5 documentation: Pi SPI and core clock (core_freq_min)
# More details: README.md of this package

import ctypes
import fcntl
import os
import struct
from typing import List, Tuple

try:
    from rpi_ws281x import Color, PixelStrip
except ImportError:  # Hardware library is only expected on the Raspberry Pi.
    Color = None
    PixelStrip = None


Rgb = Tuple[int, int, int]


class GroveWs2813RgbStick:
    """Own the `rpi_ws281x` PixelStrip object and expose RGB tuple helpers."""

    def __init__(
        self,
        led_count: int,
        gpio_pin: int,
        brightness: int = 64,
        freq_hz: int = 800000,
        dma: int = 10,
        invert: bool = False,
        channel: int = 0,
    ) -> None:
        if PixelStrip is None or Color is None:
            raise RuntimeError("Python package rpi_ws281x is not installed")

        self.led_count = led_count
        self.gpio_pin = gpio_pin
        self.strip = PixelStrip(
            led_count,
            gpio_pin,
            freq_hz,
            dma,
            invert,
            brightness,
            channel,
        )
        # Cache the last frame to avoid writing the same LED data repeatedly.
        self.last_pixels: List[Rgb] = []

    def begin(self) -> None:
        self.strip.begin()

    def clear(self) -> None:
        self.show_solid((0, 0, 0))

    def show_solid(self, color: Rgb) -> None:
        self.show_pixels([color] * self.led_count)

    def show_pixels(self, pixels: List[Rgb]) -> None:
        if len(pixels) != self.led_count:
            raise ValueError(
                f"Expected {self.led_count} RGB pixels, got {len(pixels)}"
            )
        if pixels == self.last_pixels:
            return
        self.last_pixels = list(pixels)

        for index, (red, green, blue) in enumerate(pixels):
            self.strip.setPixelColor(index, Color(red, green, blue))
        self.strip.show()


class SpiWs2813RgbStick:
    """Drive the WS2813 stick from the SPI MOSI pin (BCM GPIO10).

    rpi_ws281x does not support the Raspberry Pi 5, whose GPIO sits behind
    the RP1 chip, but SPI does. At 2.5 MHz one SPI bit lasts 400 ns, so each
    WS2813 bit is sent as three SPI bits: 110 for a one (800 ns high) and 100
    for a zero (400 ns high), inside the WS2813 timing windows. Trailing zero
    bytes hold the line low for the >280 us latch.

    The frame is sent as one SPI_IOC_MESSAGE made of <=63-byte transfers. On
    the Pi 5 (RP1, Ubuntu raspi kernel) any transfer longer than the 64-byte
    FIFO goes through DMA, which times out (-110); FIFO-sized transfers work.
    Transfers inside one message follow each other within microseconds, far
    below the WS2813's 280 us latch, so the strip sees one continuous frame.
    63 bytes is 21 colour bytes, so no LED's bits are split across transfers.
    """

    SPI_HZ = 2_500_000
    LATCH_BYTES = 120            # 120 x 3.2 us = 384 us low
    CHUNK = 63
    _XFER = struct.Struct("<QQIIHBBBBBB")   # struct spi_ioc_transfer, 32 bytes
    # Each colour byte becomes 24 SPI bits (3 bytes); precompute all 256.
    _ENCODED = []
    for _value in range(256):
        _bits = 0
        for _i in range(7, -1, -1):
            _bits = (_bits << 3) | (0b110 if (_value >> _i) & 1 else 0b100)
        _ENCODED.append(_bits.to_bytes(3, "big"))
    del _value, _bits, _i

    def __init__(self, led_count: int, brightness: int = 64, bus: int = 0, device: int = 0) -> None:
        self.led_count = led_count
        self.scale = max(0, min(255, int(brightness))) / 255.0
        self.path = f"/dev/spidev{bus}.{device}"
        self.fd = None
        self.last_pixels: List[Rgb] = []

    def begin(self) -> None:
        try:
            self.fd = os.open(self.path, os.O_RDWR)
            self.last_pixels = []
            self.clear()
        except OSError as exc:
            self.close()
            raise RuntimeError(f"SPI output on {self.path} failed: {exc}") from exc

    def clear(self) -> None:
        self.show_solid((0, 0, 0))

    def show_solid(self, color: Rgb) -> None:
        self.show_pixels([color] * self.led_count)

    def show_pixels(self, pixels: List[Rgb]) -> None:
        if len(pixels) != self.led_count:
            raise ValueError(
                f"Expected {self.led_count} RGB pixels, got {len(pixels)}"
            )
        if pixels == self.last_pixels:
            return
        self.last_pixels = list(pixels)

        frame = bytearray(b"\x00")          # start from a clean low level
        for red, green, blue in pixels:
            for value in (green, red, blue):  # WS2813 expects GRB order
                frame += self._ENCODED[int(value * self.scale)]
        frame += bytes(self.LATCH_BYTES)
        self._write_chunked(bytes(frame))

    def _write_chunked(self, data: bytes) -> None:
        chunks = [data[i:i + self.CHUNK] for i in range(0, len(data), self.CHUNK)]
        buffers = [ctypes.create_string_buffer(chunk, len(chunk)) for chunk in chunks]
        request = b"".join(
            self._XFER.pack(ctypes.addressof(buf), 0, len(chunk), self.SPI_HZ,
                            0, 8, 0, 0, 0, 0, 0)
            for buf, chunk in zip(buffers, chunks))
        # SPI_IOC_MESSAGE(n) = _IOW('k', 0, char[32 * n])
        spi_ioc_message = (1 << 30) | (len(request) << 16) | (ord("k") << 8)
        fcntl.ioctl(self.fd, spi_ioc_message, request)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
