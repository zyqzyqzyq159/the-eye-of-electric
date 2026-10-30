import os
from collections import defaultdict

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "work_dbg")
# 按时间分组频谱流：fft_channel 稳定的一段时间 = 一次帧输出
streams = []
cur = None
with open(os.path.join(base, "dbg_spec.log")) as f:
    for line in f:
        parts = line.split()
        t = int(parts[0])
        ch = int(parts[1].split("=")[1])
        b = int(parts[2].split("=")[1])
        m = int(parts[3].split("=")[1])
        if cur is None or t - cur["t_last"] > 100_000 or cur["ch"] != ch:
            cur = {"ch": ch, "t0": t, "t_last": t, "bins": {}, "n": 0}
            streams.append(cur)
        cur["t_last"] = t
        cur["bins"][b] = m
        cur["n"] += 1

print(f"streams: {len(streams)}")
for s in streams:
    if s["n"] < 100:
        continue
    total = sum(s["bins"].values())
    top = sorted(s["bins"].items(), key=lambda kv: -kv[1])[:5]
    ts = f"{s['t0']/1e6:.1f}-{s['t_last']/1e6:.1f}ms"
    print(f"ch={s['ch']} t={ts:>16s} nbins={s['n']:3d} sum={total:8d} "
          f"top={[(b, m) for b, m in top]}")
