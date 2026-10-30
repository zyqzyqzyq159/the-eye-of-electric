import os
import sys

from PIL import Image, ImageDraw, ImageFont

output_dir = os.path.dirname(os.path.abspath(__file__))

CW, CH = 8, 16

FONT_CANDIDATES = [
    r"C:\Windows\Fonts\consola.ttf",
    r"C:\Windows\Fonts\lucon.ttf",
    r"C:\Windows\Fonts\cour.ttf",
    r"C:\Windows\Fonts\arial.ttf",
]

chars = {c: chr(c) for c in range(32, 127)}
chars[127] = "\u00b0"  # 度数符号，映射到自定义槽位 127


def load_font():
    for size in (14, 13, 12, 11):
        for path in FONT_CANDIDATES:
            if os.path.exists(path):
                font = ImageFont.truetype(path, size)
                yield font


def render_char(font, ch):
    img = Image.new("L", (48, 48), 0)
    draw = ImageDraw.Draw(img)
    draw.text((16, 16), ch, font=font, fill=255)
    bbox = img.getbbox()
    if bbox is None:
        return [0] * CH
    x0, y0, x1, y1 = bbox
    w, h = x1 - x0, y1 - y0
    if w > CW:
        return None
    # 平移量：字形底部落在第 13 行（基线），水平居中
    dx = x0 - (CW - w) // 2
    dy = y1 - 13
    rows = []
    for r in range(CH):
        bits = 0
        sy = r + dy
        for c in range(CW):
            sx = c + dx
            if 0 <= sx < 48 and 0 <= sy < 48 and img.getpixel((sx, sy)) >= 96:
                bits |= 1 << (7 - c)
        rows.append(bits)
    return rows


def main():
    font = None
    for candidate in load_font():
        rows_map = {}
        ok = True
        for code, ch in chars.items():
            rows = render_char(candidate, ch)
            if rows is None:
                ok = False
                break
            rows_map[code] = rows
        if ok:
            font = candidate
            break
    if font is None:
        print("ERROR: no usable font")
        sys.exit(1)

    print(f"Using font: {font.path if hasattr(font, 'path') else font}")

    with open(os.path.join(output_dir, "font_init.vh"), "w") as f:
        f.write("// 8x16 ASCII 字库 ROM 初始化（由 gen_font.py 生成）\n")
        f.write("// font_rom[c*16 + r]：字符 c 第 r 行位图，bit7 = 最左像素\n")
        for code in range(128):
            rows = rows_map.get(code, [0] * CH)
            for r, bits in enumerate(rows):
                if bits:
                    f.write(f"font_rom[{code * 16 + r}] = 8'h{bits:02X};\n")

    with open(os.path.join(output_dir, "font_hex.txt"), "w") as f:
        for code in range(128):
            rows = rows_map.get(code, [0] * CH)
            f.write("".join(f"{b:02X}" for b in rows) + "\n")

    for probe in ("A", "3", "%", "\u00b0"):
        code = ord(probe) if probe != "\u00b0" else 127
        rows = rows_map[code]
        print(f"--- '{probe}' (slot {code}) ---")
        for bits in rows:
            print("".join("#" if bits & (1 << (7 - c)) else "." for c in range(CW)))

    print("Generated font_init.vh")


if __name__ == "__main__":
    main()
