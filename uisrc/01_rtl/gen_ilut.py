import math
import os

# 电流自检 LUT：与电压表同相频、滞后 30°（模拟 cosφ=0.866 感性负载）。
# 30° 相移直接烘焙进表值——复用电压相位累加器地址，无需独立电流累加器/双端口 ROM。
#
# 【寻址约定，勿再犯】谐波表被 kθ 地址（lut_addr_a3 = 3×phase_acc）寻址，
# 所以表内容必须是"基波形状"（关于表索引 i 只有 1 次正弦），k 次谐波由地址乘 k 产生。
# 若表内生成 sin(k(2πi/N−30°))（真 k 次谐波形状），被 kθ 寻址后输出 sin(k²θ−k·30°)
# ——3 次变 9 次、5 次变 25 次，且与基波叠加超 ±2047 削顶（k9=348/k25=167、
# 基波跌至 1976，ModelSim 实测）。
# 正确表值：T_k[i] = A_k·sin(2πi/N − k·30°)，输出 = A_k·sin(k(θ−30°))，
# 即电流 = 失真电压波形整体延迟 30°：
#   合成峰值与电压同为 1842（θ−30°=90° 处 sin3θ 反相抵消），不触饱和。
#   isin   = 2047·sin(θ−30°)      ih3 = (2047/5)·sin(θ−90°)   ih5 = (2047/10)·sin(θ−150°)
#   i80/i60 表 = B/C 相不平衡注入（80%/60%），从 A 相电流表数值等比缩放，
#   保证 h3/基波 = 20%、h5/基波 = 10% 与幅值无关（各相 THD-I 读数一致）。
# 预期自检读数：V-I 相位角 ≈ −30°（±1.5°，256 点 LUT 量化 1.4°/步进）

output_dir = os.path.dirname(os.path.abspath(__file__))

N = 256
LAG = math.pi / 6.0  # 30°


def lut_values(k, mult):
    vals = []
    for i in range(N):
        th = 2 * math.pi * i / N - k * LAG
        h = mult * math.sin(th)
        r = int(math.floor(h + 0.5)) if h >= 0 else -int(math.floor(-h + 0.5))
        vals.append(max(-2048, min(2047, r)))
    return vals


def scaled(vals, num, den):
    out = []
    for h in vals:
        if h >= 0:
            r = (h * num * 2 + den) // (2 * den)
        else:
            r = -((-h) * num * 2 + den) // (2 * den)
        out.append(max(-2048, min(2047, r)))
    return out


isin = lut_values(1, 2047.0)
ih3 = lut_values(3, 2047.0 / 5.0)     # 20%
ih5 = lut_values(5, 2047.0 / 10.0)    # 10%

tables = [
    ("isin_lut", isin),
    ("ih3_lut", ih3),
    ("ih5_lut", ih5),
    ("i80_fund_lut", scaled(isin, 4, 5)),
    ("i80_h3_lut", scaled(ih3, 4, 5)),
    ("i80_h5_lut", scaled(ih5, 4, 5)),
    ("i60_fund_lut", scaled(isin, 3, 5)),
    ("i60_h3_lut", scaled(ih3, 3, 5)),
    ("i60_h5_lut", scaled(ih5, 3, 5)),
]

with open(os.path.join(output_dir, "dds_i_lut_init.vh"), "w") as f:
    f.write("// 由 gen_ilut.py 生成：电流自检表（30° 滞后烘焙），勿手改\n")
    for name, vals in tables:
        for i, h in enumerate(vals):
            f.write(f"{name}[{i}] = 12'd{h + 2048};\n")

print("Generated dds_i_lut_init.vh (9 tables, 30-deg lag baked in)")
print("spot check: isin[0] =", isin[0] + 2048, "(sin(-30deg)*2047+2048 =",
      round(2047 * math.sin(-math.pi / 6) + 2048), ")")
