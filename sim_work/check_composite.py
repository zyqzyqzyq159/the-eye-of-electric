import os, re, math

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "uisrc", "01_rtl")
tables = {}
for fname in ("dds_i_lut_init.vh",):
    with open(os.path.join(base, fname), encoding="utf-8") as f:
        for line in f:
            m = re.match(r"(\w+)\[(\d+)\]\s*=\s*12'd(\d+);", line.strip())
            if m:
                tables.setdefault(m.group(1), [0] * 256)[int(m.group(2))] = int(m.group(3))


def composite(fund, h3, h5):
    """复现 dds_signal 寻址：θ 步进，谐波表用 3θ/5θ 地址。
    三表均 2048 中心，AC 部分 = 和 − 3×2048"""
    out = []
    for th in range(256):
        v = fund[th] + h3[(3 * th) % 256] + h5[(5 * th) % 256] - 3 * 2048
        v = max(-2048, min(2047, v))  # dist_sat 等效（AC 域 ±2047）
        out.append(v)
    return out


def spec(x):
    mags = []
    for k in range(128):
        re_ = sum(x[n] * math.cos(2 * math.pi * k * n / 256) for n in range(256))
        im_ = -sum(x[n] * math.sin(2 * math.pi * k * n / 256) for n in range(256))
        mags.append(math.hypot(re_, im_) * 2 / 256)
    return mags


for label, (f, h3, h5) in {
    "A 相 100%": ("isin_lut", "ih3_lut", "ih5_lut"),
    "B 相  80%": ("i80_fund_lut", "i80_h3_lut", "i80_h5_lut"),
    "C 相  60%": ("i60_fund_lut", "i60_h3_lut", "i60_h5_lut"),
}.items():
    x = composite(tables[f], tables[h3], tables[h5])
    m = spec(x)
    peak = max(max(x), -min(x))
    h3p, h5p = m[3] / m[1] * 100, m[5] / m[1] * 100
    thd_sum = (m[3] + m[5]) / m[1] * 100
    print(f"{label}: peak={peak} (饱和 2047) 基波={m[1]:.0f} "
          f"h3={h3p:.1f}% h5={h5p:.1f}% THD求和={thd_sum:.1f}% "
          f"泄漏junk={max(m[6:]):.1f}")

# 电压对照（同幅度结构）
vs = [0] * 256
with open(os.path.join(base, "sin_lut_init.vh"), encoding="utf-8") as f:
    for line in f:
        m = re.match(r"(\w+)\[(\d+)\]\s*=\s*12'd(\d+);", line.strip())
        if m and m.group(1) == "sin_lut":
            vs[int(m.group(2))] = int(m.group(3))
h3v, h5v = [0] * 256, [0] * 256
with open(os.path.join(base, "harm3_lut_init.vh"), encoding="utf-8") as f:
    for line in f:
        m = re.match(r"(\w+)\[(\d+)\]\s*=\s*12'd(\d+);", line.strip())
        if m:
            h3v[int(m.group(2))] = int(m.group(3))
with open(os.path.join(base, "harm5_lut_init.vh"), encoding="utf-8") as f:
    for line in f:
        m = re.match(r"(\w+)\[(\d+)\]\s*=\s*12'd(\d+);", line.strip())
        if m:
            h5v[int(m.group(2))] = int(m.group(3))
xv = composite(vs, h3v, h5v)
mv = spec(xv)
print(f"电压对照: peak={max(max(xv), -min(xv))} 基波={mv[1]:.0f} "
      f"h3={mv[3]/mv[1]*100:.1f}% h5={mv[5]/mv[1]*100:.1f}%")

# V-I 相位差：基波相位角（用最小二乘拟合 sin/cos 分量）
import cmath


def fund_phase(x):
    re_ = sum(x[n] * math.cos(2 * math.pi * n / 256) for n in range(256))
    im_ = -sum(x[n] * math.sin(2 * math.pi * n / 256) for n in range(256))
    return math.degrees(math.atan2(im_, re_))


pv = fund_phase(xv)
for label, (f, h3, h5) in {
    "A": ("isin_lut", "ih3_lut", "ih5_lut"),
    "B80": ("i80_fund_lut", "i80_h3_lut", "i80_h5_lut"),
    "C60": ("i60_fund_lut", "i60_h3_lut", "i60_h5_lut"),
}.items():
    pi_ = fund_phase(composite(tables[f], tables[h3], tables[h5]))
    print(f"V-I 相位差 ({label}): {pi_ - pv:+.1f}° (期望 -30°)")
