import zlib, struct, os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

def read_ppm(path):
    with open(path, 'rb') as f:
        data = f.read()
    parts = data.split(b'\n', 3)
    assert parts[0] == b'P6', parts[0]
    w, h = map(int, parts[1].split())
    assert parts[2] == b'255'
    return w, h, parts[3][:w * h * 3]

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

def crop(rgb, w, x0, y0, x1, y1):
    return b''.join(rgb[(y * w + x0) * 3:(y * w + x1) * 3] for y in range(y0, y1))

w, h, rgb = read_ppm('display_frame.ppm')
write_png('display_frame.png', w, h, rgb)
write_png('crop_ylabels.png', 150, h - 40, crop(rgb, w, 1130, 40, 1280, h))    # 右缘 Y 标签区
write_png('crop_xlabels.png', w - 200, h - 660, crop(rgb, w, 200, 660, w, h))  # 底部 X 标签区
write_png('crop_windowA.png', 600, 220, crop(rgb, w, 200, 40, 800, 260))       # A 窗网格+波形
print('PNG ok:', os.path.getsize('display_frame.png'), 'bytes')
