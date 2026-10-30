import os, math

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "work_dbg")
events = []
with open(os.path.join(base, "dbg_fsamp.log")) as f:
    for line in f:
        parts = line.split()
        events.append((int(parts[0]), int(parts[1]), int(parts[2])))

win = [d for t, ch, d in events if ch == 3 and 119_900_000 < t < 140_050_000][:256]
x = [v - 2048 for v in win]
ref = {}
for k in range(128):
    re_ = sum(x[i] * math.cos(2 * math.pi * k * i / 256) for i in range(256))
    im_ = -sum(x[i] * math.sin(2 * math.pi * k * i / 256) for i in range(256))
    ref[k] = math.hypot(re_, im_) * 2 / 256 * 129

# 硬件 ch3 流（t 在 140.1ms 附近）
hw = {}
with open(os.path.join(base, "dbg_spec.log")) as f:
    for line in f:
        parts = line.split()
        t = int(parts[0])
        ch = int(parts[1].split("=")[1])
        b = int(parts[2].split("=")[1])
        m = int(parts[3].split("=")[1])
        if ch == 3 and 140_000_000 < t < 141_000_000:
            hw[b] = m

print(f"{'bin':>4} {'hw':>8} {'ref':>8} {'diff':>8}")
for b in range(128):
    if hw.get(b, 0) > 300 or ref[b] > 300 or abs(hw.get(b, 0) - ref[b]) > 300:
        print(f"{b:>4} {hw.get(b,0):>8} {ref[b]:>8.0f} {hw.get(b,0)-ref[b]:>8.0f}")
print(f"\nhw sum={sum(hw.values())}  ref sum={sum(ref.values()):.0f}")
print(f"hw DC={hw.get(0)}  ref DC={ref[0]:.0f}")
