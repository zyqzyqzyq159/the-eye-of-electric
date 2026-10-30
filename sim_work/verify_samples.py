import os, re

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "work_dbg")

# 读表
vh = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "..", "uisrc", "01_rtl", "dds_i_lut_init.vh")
tables = {}
with open(vh, encoding="utf-8") as f:
    for line in f:
        m = re.match(r"(\w+)\[(\d+)\]\s*=\s*12'd(\d+);", line.strip())
        if m:
            tables.setdefault(m.group(1), [0] * 256)[int(m.group(2))] = int(m.group(3))

# 读日志
events = []
with open(os.path.join(base, "dbg_fsamp.log")) as f:
    for line in f:
        parts = line.split()
        events.append((int(parts[0]), int(parts[1]), int(parts[2])))

# ch3 装载窗口 = pulses 1536-1791 = 时间 [120.08, 140.01]ms
# pulse N 时刻 = 1us + (N+1)*78.13us
win = [(t, d) for t, ch, d in events if ch == 3 and 120_050_000 < t < 140_020_000]
print(f"window: {len(win)} samples, t {win[0][0]/1e6:.3f}..{win[-1][0]/1e6:.3f} ms")

isin = tables["isin_lut"]
mismatch = 0
for i, (t, d) in enumerate(win[:256]):
    # 装载窗口起点 pulse 1536 -> 表索引 0（推算），对照后续是否逐点吻合
    if d != isin[i]:
        mismatch += 1
        if mismatch <= 12:
            print(f"  sample {i:3d} t={t/1e6:.3f}ms logged={d} table[{i}]={isin[i]} "
                  f"diff={d-isin[i]}")
print(f"mismatches vs isin_lut[0..255] sequential: {mismatch}/256")

# 试试错位一格（窗口可能偏 1 个样本）
for off in (-2, -1, 1, 2):
    mm = sum(1 for i in range(256) if 0 <= i + off < 256 and win[i][1] != isin[i + off])
    print(f"  offset {off:+d}: {mm} mismatches")
