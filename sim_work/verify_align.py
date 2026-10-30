import cmath

# 复刻硬件: 100MHz, DDS 50Hz, 256-entry LUT, 每7813周期采样一次
CLK = 100_000_000
SAMP_DIV = 7813
PHASE_INCR = 2147          # (50*10995+128)>>8, 与 dds_signal.v 一致
LUT_BITS = 256
ACC_BITS = 32
PHASE_120 = 0x55555555
PHASE_240 = 0xAAAAAAAA

acc_a, acc_b, acc_c = 0, PHASE_240, PHASE_120
lut = [int(round(2047 * cmath.sin(2*cmath.pi*i/LUT_BITS).real)) + 2048 for i in range(LUT_BITS)]

# 逐周期推进到采样点，记录三相样本
samples = {0: [], 1: [], 2: []}
for pulse in range(0, 1800):
    for _ in range(SAMP_DIV):
        acc_a = (acc_a + PHASE_INCR) % (1 << ACC_BITS)
        acc_b = (acc_b + PHASE_INCR) % (1 << ACC_BITS)
        acc_c = (acc_c + PHASE_INCR) % (1 << ACC_BITS)
    samples[0].append(lut[acc_a >> 24])
    samples[1].append(lut[acc_b >> 24])
    samples[2].append(lut[acc_c >> 24])

def fft_bin1(frame):
    N = len(frame)
    X = sum((v - 2048) * cmath.exp(-2j*cmath.pi*1*k/N) for k, v in enumerate(frame))
    return X

# 帧对齐: 帧起点 = 512 的倍数脉冲 (A:0, B:512, C:1024, A2:1536)
frames = {}
for ch, start in [(0, 0), (1, 512), (2, 1024), (0, 1536)]:
    frames.setdefault(ch, []).append(fft_bin1(samples[ch][start:start+256]))

def ang(X):
    a = cmath.phase(X) * 65536 / (2*cmath.pi)
    return a % 65536

phA1 = ang(frames[0][0]); phB = ang(frames[1][0]); phC = ang(frames[2][0]); phA2 = ang(frames[0][1])

def s16(x):
    return x - 65536 if x >= 32768 else x

print(f"phA(frame1)={phA1:.0f}  phB={phB:.0f}  phC={phC:.0f}  phA(frame2)={phA2:.0f}")
print(f"phase_ab = {s16((phB-phA1)%65536):.0f}  (期望 -21845 = -120deg)")
print(f"phase_bc = {s16((phC-phB)%65536):.0f}")
print(f"phase_ca = {s16((phA2-phC)%65536):.0f}")

# 对照: 无 LUT 量化的理想正弦（连续相位直接采样）
def ideal_ph(ch_off, start_pulse):
    acc = (ch_off + start_pulse*SAMP_DIV*PHASE_INCR) % (1 << ACC_BITS)
    X = 0+0j
    for k in range(256):
        acc = (acc + SAMP_DIV*PHASE_INCR) % (1 << ACC_BITS)
        X += cmath.sin(2*cmath.pi*acc/(1<<ACC_BITS)) * cmath.exp(-2j*cmath.pi*k/256)
    return ang(X)

iA = ideal_ph(0, 0); iB = ideal_ph(PHASE_240, 512); iC = ideal_ph(PHASE_120, 1024)
print(f"\n理想(无LUT量化): phase_ab = {s16((iB-iA)%65536):.0f}  phase_bc = {s16((iC-iB)%65536):.0f}")
