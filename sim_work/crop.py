import zlib, struct, os, sys

# 裁剪 PPM 区域并放大 N 倍存 PNG：crop.py <src.ppm> <x0> <y0> <w> <h> <scale> <out.png>
src, x0, y0, w, h, scale, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6]), sys.argv[7]

with open(src, 'rb') as f:
    data = f.read()
parts = data.split(b'\n', 3)
W, H = map(int, parts[1].split())
rgb = parts[3][:W * H * 3]

def px(x, y):
    i = (y * W + x) * 3
    return rgb[i], rgb[i+1], rgb[i+2]

def write_png(path, w, h, rgb):
    def chunk(tag, dat):
        c = tag + dat
        return struct.pack('>I', len(dat)) + c + struct.pack('>I', zlib.crc32(c) & 0xFFFFFFFF)
    raw = b''.join(b'\x00' + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9))
           + chunk(b'IEND', b''))
    with open(path, 'wb') as f:
        f.write(png)

buf = bytearray()
for y in range(h):
    for _ in range(scale):
        row = bytearray()
        for x in range(w):
            r, g, b = px(x0 + x, y0 + y)
            for _ in range(scale):
                row += bytes((r, g, b))
        buf += row
write_png(out, w * scale, h * scale, bytes(buf))
print('crop ok:', out, w * scale, 'x', h * scale)
