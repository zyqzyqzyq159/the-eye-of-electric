import re, os, sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))

font = {}
for line in open(r'..\uisrc\01_rtl\font_init.vh'):
    m = re.match(r"font_rom\[(\d+)\]\s*=\s*8'h([0-9A-Fa-f]+)", line.strip())
    if m:
        font[int(m.group(1))] = int(m.group(2), 16)

d = open('display_frame.ppm', 'rb').read().split(b'\n', 3)
w, h = map(int, d[1].split())
px = d[3][:w * h * 3]

def pix(x, y):
    i = (y * w + x) * 3
    return (px[i] << 16) | (px[i + 1] << 8) | px[i + 2]

BG = 0x0A0A12

def check_label(x0, y0, s, skip_tick=False):
    ok = True
    for i, ch in enumerate(s):
        for r in range(16):
            bits = font.get(ord(ch) * 16 + r, 0)
            for c in range(8):
                x = x0 + i * 8 + c
                y = y0 + r
                v = pix(x, y)
                exp_on = (bits >> (7 - c)) & 1
                if skip_tick and x == 1240 and 700 <= y <= 703:
                    act_on = (v == 0xC8C8DC)     # 刻度短线穿过空白区，只认字形色
                else:
                    act_on = (v != BG)
                if exp_on != act_on:
                    print('MISMATCH %r char %r (%d,%d) exp=%d got=%06X' % (s, ch, x, y, exp_on, v))
                    ok = False
    return ok

labels = [
    (1240,  64, '1.65V', False),
    (1240, 144, '0V',    False),
    (1240, 240, '-1.65', False),
    (1240, 288, '1.65V', False),
    (1240, 368, '0V',    False),
    (1240, 464, '-1.65', False),
    (1240, 512, '1.65V', False),
    (1240, 608, '0V',    False),
    (1240, 688, '-1.65', True),     # '-' 格穿过 x1240 刻度短线
    (216,  704, '0',     False),
    (464,  704, '10',    False),
    (720,  704, '20',    False),
    (976,  704, '30',    False),
    (1232, 704, '40',    False),
    (1256, 704, 'ms',    False),
]

fails = 0
for x0, y0, s, st in labels:
    if check_label(x0, y0, s, st):
        print('PASS  (%d,%d) %r' % (x0, y0, s))
    else:
        print('FAIL  (%d,%d) %r' % (x0, y0, s))
        fails += 1
sys.exit(1 if fails else 0)
