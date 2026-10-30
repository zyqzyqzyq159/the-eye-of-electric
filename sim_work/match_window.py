import os, math

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "work_dbg")

# 1) 读入全部样本事件 (t, ch, data)
events = []
with open(os.path.join(base, "dbg_fsamp.log")) as f:
    for line in f:
        parts = line.split()
        events.append((int(parts[0]), int(parts[1]), int(parts[2])))

print(f"total sample events: {len(events)}")

# 2) 检查脉冲间隔是否均匀（ch3 段）
gaps = []
prev_t = None
for t, ch, d in events:
    if ch == 3:
        if prev_t is not None:
            gaps.append(t - prev_t)
        prev_t = t
from collections import Counter
print("ch3 pulse gap histogram:", Counter(gaps).most_common(5))

# 3) 硬件 ch3@140.1ms 谱（来自 spec log）：bin1=262336, DC=523827, junk~520
# 滑窗：在 ch3 样本序列里找 256 长窗口，其 DFT bin1 最接近 262336 且 DC 最接近 523827
ch3 = [(t, d) for t, ch, d in events if ch == 3]
print(f"ch3 samples: {len(ch3)}")


def dft_bins(x):
    n = len(x)
    out = {}
    for k in [0, 1, 3, 5, 69, 111, 119]:
        re_ = sum(x[i] * math.cos(2 * math.pi * k * i / n) for i in range(n))
        im_ = -sum(x[i] * math.sin(2 * math.pi * k * i / n) for i in range(n))
        out[k] = math.hypot(re_, im_) * 2 / n * 129  # 链路标度
    return out


best = None
for start in range(0, len(ch3) - 255):
    x = [d - 2048 for _, d in ch3[start:start + 256]]
    # 先粗筛：bin1
    re1 = sum(x[i] * math.cos(2 * math.pi * i / 256) for i in range(256))
    im1 = -sum(x[i] * math.sin(2 * math.pi * i / 256) for i in range(256))
    b1 = math.hypot(re1, im1) * 2 / 256 * 129
    dc = sum(x) / 256 * 129 + 524288
    score = abs(b1 - 262336) + abs(dc - 523827) / 10
    if best is None or score < best[0]:
        best = (score, start, b1, dc)
print(f"best window start={best[1]} (t={ch3[best[1]][0]/1e6:.2f}ms) "
      f"bin1={best[2]:.0f} dc={best[3]:.0f} score={best[0]:.0f}")
print("window DFT:", {k: round(v) for k, v in dft_bins(
    [d - 2048 for _, d in ch3[best[1]:best[1] + 256]]).items()})
