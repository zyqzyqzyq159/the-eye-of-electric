import math
import os

output_dir = os.path.dirname(os.path.abspath(__file__))

LUT_SIZE = 1024
DAC_BITS = 12

sin_lut = []
for i in range(LUT_SIZE):
    val = int((2.0**(DAC_BITS-1) - 1) * math.sin(2.0 * math.pi * i / LUT_SIZE) + 2.0**(DAC_BITS-1))
    val = max(0, min(4095, val))
    sin_lut.append(val)

with open(os.path.join(output_dir, "sin_lut.hex"), "w") as f:
    for val in sin_lut:
        f.write(f"{val:03X}\n")

with open(os.path.join(output_dir, "sin_lut_init.vh"), "w") as f:
    for i, val in enumerate(sin_lut):
        f.write(f"sin_lut[{i}] = 12'h{val:03X};\n")

print(f"Generated sin_lut.hex and sin_lut_init.vh with {LUT_SIZE} entries")

FFT_SIZE = 1024
HALF_FFT = FFT_SIZE // 2

tw_real = []
tw_imag = []
for i in range(HALF_FFT):
    re = int(32767.0 * math.cos(2.0 * math.pi * i / FFT_SIZE))
    im = int(-32767.0 * math.sin(2.0 * math.pi * i / FFT_SIZE))
    re = max(-32768, min(32767, re))
    im = max(-32768, min(32767, im))
    tw_real.append(re)
    tw_imag.append(im)

with open(os.path.join(output_dir, "twiddle_real.hex"), "w") as f:
    for val in tw_real:
        v = val + 65536 if val < 0 else val
        f.write(f"{v:04X}\n")

with open(os.path.join(output_dir, "twiddle_imag.hex"), "w") as f:
    for val in tw_imag:
        v = val + 65536 if val < 0 else val
        f.write(f"{v:04X}\n")

with open(os.path.join(output_dir, "twiddle_real_init.vh"), "w") as f:
    for i, val in enumerate(tw_real):
        v = val + 65536 if val < 0 else val
        f.write(f"twiddle_rom_real[{i}] = 16'h{v:04X};\n")

with open(os.path.join(output_dir, "twiddle_imag_init.vh"), "w") as f:
    for i, val in enumerate(tw_imag):
        v = val + 65536 if val < 0 else val
        f.write(f"twiddle_rom_imag[{i}] = 16'h{v:04X};\n")

print(f"Generated twiddle hex and vh files with {HALF_FFT} entries")
print("All ROM files generated successfully!")