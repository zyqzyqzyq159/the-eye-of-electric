import os, math

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "work_dbg")
frames = []  # (chan, [samples])
cur = None
with open(os.path.join(base, "dbg_fsamp.log")) as f:
    for line in f:
        t, ch, d = line.split()
        ch, d = int(ch), int(d)
        if cur is None or cur[0] != ch or len(cur[1]) >= 256:
            cur = (ch, [])
            frames.append(cur)
        cur[1].append(d)

print(f"total frames: {len(frames)}")
for i, (ch, s) in enumerate(frames):
    if len(s) != 256:
        continue
    x = [v - 2048 for v in s]
    mags = []
    for k in range(128):
        re_ = sum(x[n] * math.cos(2 * math.pi * k * n / 256) for n in range(256))
        im_ = -sum(x[n] * math.sin(2 * math.pi * k * n / 256) for n in range(256))
        mags.append(math.hypot(re_, im_) * 2 / 256)
    total = sum(mags)
    top3 = sorted(range(1, 128), key=lambda k: -mags[k])[:4]
    desc = " ".join(f"k{k}={mags[k]:.0f}" for k in top3)
    print(f"frame{i:3d} ch={ch} sum={total:8.1f} peak={max(x):5d}/{min(x):6d} {desc}")
