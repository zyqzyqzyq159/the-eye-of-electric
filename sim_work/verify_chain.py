# -*- coding: utf-8 -*-
# 数值验证自检链路：DDS LUT -> 7813 分频采样 -> 256 点 FFT -> THD/相位
# 目的：确认 (1) 功能上 THD 应该接近 0；(2) 通道帧错位导致的相位读数
import re, cmath

# 读取 sin LUT
lut = []
with open(r"c:\Users\hp\Desktop\FPGA\AL_TRY\uisrc\01_rtl\sin_lut_init.vh") as f:
    for line in f:
        m = re.search(r"12'd(\d+)", line)
        if m:
            lut.append(int(m.group(1)))
assert len(lut) == 256, len(lut)

PHASE_INCR = 2147            # DDS 相位增量（top: freq=50Hz 的舍入结果）
SAMPLE_DIV = 7813            # 采样分频（100MHz/7813 = 12799.18Hz）
PH_120 = (1 << 32) // 3      # C 相偏移 (+120deg)
PH_240 = (2 << 32) // 3      # B 相偏移 (+240deg)

# 模拟采样：相位累加器每时钟 +2147，每 7813 个时钟采一次
def sample_frames(start_clock, nframes, offset):
    """返回 nframes 帧，每帧 256 点，帧起始时钟间隔由 FFT 处理时间决定"""
    frames = []
    t = start_clock
    for _ in range(nframes):
        smp = []
        for i in range(256):
            clk = t + i * SAMPLE_DIV
            ph = (offset + PHASE_INCR * clk) & 0xFFFFFFFF
            smp.append(lut[ph >> 24])
        frames.append(smp)
        t += offset_step  # 帧间隔（由 FSM 决定）
    return frames

# ---- 帧间隔分析 ----
# A 帧: 脉冲 P0..P255；FFT 处理 ~9731 clk = 97.3us > 78.13us(采样周期)
# 处理在 P256 之后、P257 之前完成 -> 下一帧从 P257 开始（帧间隔 = 257 个采样脉冲）
offset_step = 257 * SAMPLE_DIV

frames_a = sample_frames(0, 4, 0)
frames_b = sample_frames(0, 4, PH_240)
frames_c = sample_frames(0, 4, PH_120)

def fft_mag_phase(frame):
    N = 256
    out = []
    for k in range(N // 2):
        s = 0
        for n, x in enumerate(frame):
            s += (x - 2048) * cmath.exp(-2j * cmath.pi * k * n / N)
        out.append(s)
    return out

def analyze(frame):
    X = fft_mag_phase(frame)
    fund = abs(X[1])
    mags = [abs(X[k]) for k in range(16)]
    h2 = mags[2] / fund * 100
    thd = sum(mags[k] / fund * 100 for k in range(2, 16))
    ph = cmath.phase(X[1])
    return fund, h2, thd, ph, X

print("=== 各相频谱（帧间隔 257 采样脉冲，即当前硬件行为） ===")
for name, fr in (("A", frames_a), ("B", frames_b), ("C", frames_c)):
    fund, h2, thd, ph, X = analyze(fr[1])
    print(f"{name}: fund={fund:9.1f}  H2={h2:6.3f}%  sumH2..15={thd:6.3f}%  ph1={ph*180/cmath.pi:8.2f}deg")

# 相位差（跨帧，帧间隔 257 脉冲）
_, _, _, phA, _ = analyze(frames_a[1])
_, _, _, phB, _ = analyze(frames_b[1])
_, _, _, phC, _ = analyze(frames_c[1])
def fold(d):
    while d > 180: d -= 360
    while d < -180: d += 360
    return d
print(f"\nphase_ab = {fold((phB-phA)*180/cmath.pi):8.2f} deg  (期望 -120)")
print(f"phase_bc = {fold((phC-phB)*180/cmath.pi):8.2f} deg")
print(f"phase_ca = {fold((phA-phC)*180/cmath.pi):8.2f} deg")

# 不同帧间隔 k 对 phase_ab 的影响
f_dds = PHASE_INCR * 1e8 / 2**32
fs = 1e8 / SAMPLE_DIV
dphi = 360 * f_dds / fs
print(f"\nf_dds = {f_dds:.5f} Hz, fs = {fs:.3f} Hz, 每采样脉冲相位步进 = {dphi:.5f} deg")
for k in (256, 257, 258, 259, 260, 255, 254):
    print(f"  帧间隔 {k} 脉冲 -> phase_ab = {fold(-120 + k*dphi):8.2f} deg")

# ---- 快照方案（三相同帧） ----
print("\n=== 快照方案（A/B/C 同一帧窗口） ===")
_, _, _, phA2, _ = analyze(frames_a[1])
_, _, _, phB2, _ = analyze(frames_b[1])   # 同一窗口
# 快照方案下 B/C 帧与 A 帧完全同窗口
frames_b_snap = sample_frames(0, 2, PH_240)
frames_c_snap = sample_frames(0, 2, PH_120)
_, _, _, phB2, _ = analyze(frames_b_snap[1])
_, _, _, phC2, _ = analyze(frames_c_snap[1])
print(f"phase_ab = {fold((phB2-phA2)*180/cmath.pi):8.2f} deg  (期望 -120)")
print(f"phase_bc = {fold((phC2-phB2)*180/cmath.pi):8.2f} deg")
print(f"phase_ca = {fold((phA2-phC2)*180/cmath.pi):8.2f} deg")

# ---- DDS 频率失配本身产生的泄漏（无帧错位影响） ----
print("\n=== 纯频谱泄漏检查（单帧 FFT，与帧错位无关） ===")
for k in (0, 1, 2):
    fr = sample_frames(k * SAMPLE_DIV // 2, 1, 0)[0]
    fund, h2, thd, ph, X = analyze(fr)
    harm = [abs(X[j])/fund*100 for j in range(2, 8)]
    print(f"起始相位偏移 {k}: H2={h2:.4f}%  H2..H7={['%.3f'%h for h in harm]}")
