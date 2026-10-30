import math, re, os

# 解析 dds_i_lut_init.vh，对每个表做 256 点相干 FFT，
# 复现 measurement 的 rms_sum（全部 bin 幅度和）与谐波比例
vh = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                  "..", "uisrc", "01_rtl", "dds_i_lut_init.vh")
tables = {}
with open(vh, encoding="utf-8") as f:
    for line in f:
        m = re.match(r"(\w+)\[(\d+)\]\s*=\s*12'd(\d+);", line.strip())
        if m:
            name, idx, val = m.group(1), int(m.group(2)), int(m.group(3))
            tables.setdefault(name, [0] * 256)[idx] = val

N = 256


def spectrum(sig):
    # sig: 原始 12bit 码（含 2048 偏置），去偏置后 FFT
    x = [v - 2048 for v in sig]
    mags = []
    for k in range(N):
        re_ = sum(x[n] * math.cos(2 * math.pi * k * n / N) for n in range(N))
        im_ = -sum(x[n] * math.sin(2 * math.pi * k * n / N) for n in range(N))
        # fft_top 输出幅度 = 2|X[k]|/N × ... 按测量链比例：fund_mag(2047 峰)≈262074
        mags.append(math.hypot(re_, im_) * 2 / N)
    return mags


for name, sig in tables.items():
    mags = spectrum(sig)
    rms_sum = sum(mags)
    fund = max(mags[1:], default=0)
    fund_k = mags[1:].index(fund) + 1 if fund else 0
    h3 = mags[3] if len(mags) > 3 else 0
    h5 = mags[5] if len(mags) > 5 else 0
    print(f"{name:14s} peak={max(abs(v-2048) for v in sig):5d} "
          f"dc={mags[0]:7.1f} fund(k={fund_k})={fund:7.1f} "
          f"h3={h3:6.1f}({h3/fund*100 if fund else 0:5.1f}%) "
          f"h5={h5:6.1f}({h5/fund*100 if fund else 0:5.1f}%) "
          f"sum={rms_sum:8.1f}")

# 链路换算参考：纯 2047 峰正弦 → rms_sum≈264433（含 1.0088 因子）
# 失真合成(1+0.2+0.1) → ≈343700；i80 → ×0.8；i60 → ×0.6
