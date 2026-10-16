"""5x7 pixel font and an SSD1306 page-format framebuffer.

Drawing straight into the controller's page layout keeps the text crisp at
any integer scale and removes the dependency on Pillow's default font, whose
size changed between Pillow releases (10.1+ returns an 11 px FreeType face).
"""

# References (full entries: docs/references.md at the repository root):
#   [R78] SSD1306 datasheet: SSD1306 page format
# More details: README.md of this package

# Classic 5x7 ASCII font, 0x20..0x7E. Five column bytes per glyph, bit 0 = top.
_GLYPHS = bytes.fromhex(
    '0000000000' '00005f0000' '0007000700' '147f147f14' '242a7f2a12'  # sp ! " # $
    '2313086462' '3649552250' '0005030000' '001c224100' '0041221c00'  # % & ' ( )
    '14083e0814' '08083e0808' '0050300000' '0808080808' '0060600000'  # * + , - .
    '2010080402' '3e5149453e' '00427f4000' '4261514946' '2141454b31'  # / 0 1 2 3
    '1814127f10' '2745454539' '3c4a494930' '0171090503' '3649494936'  # 4 5 6 7 8
    '064949291e' '0036360000' '0056360000' '0814224100' '1414141414'  # 9 : ; < =
    '0041221408' '0201510906' '324979413e' '7e1111117e' '7f49494936'  # > ? @ A B
    '3e41414122' '7f4141221c' '7f49494941' '7f09090901' '3e4149497a'  # C D E F G
    '7f0808087f' '00417f4100' '2040413f01' '7f08142241' '7f40404040'  # H I J K L
    '7f020c027f' '7f0408107f' '3e4141413e' '7f09090906' '3e4151215e'  # M N O P Q
    '7f09192946' '4649494931' '01017f0101' '3f4040403f' '1f2040201f'  # R S T U V
    '3f4038403f' '6314081463' '0708700807' '6151494543' '007f414100'  # W X Y Z [
    '0204081020' '0041417f00' '0402010204' '4040404040' '0001020400'  # \ ] ^ _ `
    '2054545478' '7f48444438' '3844444420' '384444487f' '3854545418'  # a b c d e
    '087e090102' '0c5252523e' '7f08040478' '00447d4000' '2040443d00'  # f g h i j
    '7f10284400' '00417f4000' '7c04180478' '7c08040478' '3844444438'  # k l m n o
    '7c14141408' '081414187c' '7c08040408' '4854545420' '043f444020'  # p q r s t
    '3c4040207c' '1c2040201c' '3c4030403c' '4428102844' '0c5050503c'  # u v w x y
    '4464544c44' '0008364100' '00007f0000' '0041360800' '1008081008'  # z { | } ~
)


class Framebuffer:
    """Monochrome buffer in SSD1306 vertical-page byte order."""

    def __init__(self, width, height):
        if height % 8:
            raise ValueError('height must be a multiple of 8')
        self.width, self.height = width, height
        self.buffer = bytearray(width * height // 8)

    def pixel(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.buffer[(y // 8) * self.width + x] |= 1 << (y % 8)

    def text(self, x, y, string, scale=1):
        """Draw ASCII text; unknown characters render as '?'."""
        for char in string:
            code = ord(char)
            index = code - 0x20 if 0x20 <= code <= 0x7E else ord('?') - 0x20
            for column in range(5):
                bits = _GLYPHS[index * 5 + column]
                for row in range(7):
                    if bits >> row & 1:
                        for dx in range(scale):
                            for dy in range(scale):
                                self.pixel(x + column * scale + dx, y + row * scale + dy)
            x += 6 * scale

    def rotated_180(self):
        """Return the buffer turned upside down, for a display mounted inverted."""
        return bytes(int(f'{b:08b}'[::-1], 2) for b in reversed(self.buffer))
