import zlib, struct, os, sys, glob

target = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
os.chdir(target)

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

import glob
for ppm in sorted(glob.glob('ui_*.ppm')):
    name = ppm[:-4]
    w, h, rgb = read_ppm(ppm)
    write_png(name + '.png', w, h, rgb)
    print('PNG ok:', name + '.png', os.path.getsize(name + '.png'), 'bytes')
