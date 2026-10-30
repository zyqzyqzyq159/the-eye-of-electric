import os, math

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "work_dbg")
events = []
with open(os.path.join(base, "dbg_fsamp.log")) as f:
    for line in f:
        parts = line.split()
        events.append((int(parts[0]), int(parts[1]), int(parts[2])))

# 硬件 ch3 帧窗口：pulses 1536-1791 = [120.0, 140.0]ms
win = [(t, d) for t, ch, d in events if ch == 3 and 119_900_000 < t < 140_050_000]
print(f"window samples: {len(win)}  t: {win[0][0]/1e6:.3f}..{win[-1][0]/1e6:.3f}ms")

x = [d - 2048 for _, d in win[:256]]
print("first 16:", [d for _, d in win[:16]])
print("mean(DC):", sum(x) / 256)
mags = {}
for k in range(128):
    re_ = sum(x[i] * math.cos(2 * math.pi * k * i / 256) for i in range(256))
    im_ = -sum(x[i] * math.sin(2 * math.pi * k * i / 256) for i in range(256))
    mags[k] = math.hypot(re_, im_) * 2 / 256 * 129
total = sum(mags.values())
top = sorted(range(1, 128), key=lambda k: -mags[k])[:6]
print(f"sum={total:.0f} (hardware STORE: 321560)")
print(f"DC={mags[0]:.0f} (hardware: 523827)")
print("top bins:", [(k, round(mags[k])) for k in top])
print("hardware junk bins: 69:520 111:522 119:528")

# 相邻样本相位推进检查：用反正切跟踪相邻样本间的相位差
# 若采样相干，相邻样本相位差恒定 ~ 2π/256
prev = None
diffs = []
for i in range(0, 255):
    a = math.atan2(-(win[i+1][1] - 2048), 0) if False else None
# 简化：直接看样本值序列的过零间隔
vals = [d for _, d in win[:256]]
print("zero-crossing indices (rising through 2048):",
      [i for i in range(1, 256) if vals[i-1] < 2048 <= vals[i]])
