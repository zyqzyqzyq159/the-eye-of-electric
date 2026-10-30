import os
import re

# 从现有 sin_lut_init.vh 派生谐波表：harm3 = 基波/5（20%），harm5 = 基波/10（10%）。
# 直接读取基波表数值而非重新计算，保证谐波与基波幅度成精确比例；
# 合成信号理论 THD = sqrt(0.2^2 + 0.1^2) = 22.36%（标准均方根定义）。

output_dir = os.path.dirname(os.path.abspath(__file__))

vals = {}
with open(os.path.join(output_dir, "sin_lut_init.vh")) as f:
    for line in f:
        m = re.search(r"sin_lut\[(\d+)\]\s*=\s*12'([dh])([0-9A-Fa-f]+)", line)
        if m:
            vals[int(m.group(1))] = int(m.group(3), 16 if m.group(2) == "h" else 10)

assert len(vals) == 256 and vals[0] == 2048, "sin_lut_init.vh 应为 256 项、2048 中心"


def scaled(v, num, den):
    # 对称四舍五入：2048 + round((v-2048)*num/den)，并夹紧到 0..4095
    h = v - 2048
    if h >= 0:
        r = (h * num * 2 + den) // (2 * den)
    else:
        r = -((-h) * num * 2 + den) // (2 * den)
    return max(0, min(4095, 2048 + r))


with open(os.path.join(output_dir, "harm3_lut_init.vh"), "w") as f:
    for i in range(256):
        f.write(f"harm3_lut[{i}] = 12'd{scaled(vals[i], 1, 5)};\n")

with open(os.path.join(output_dir, "harm5_lut_init.vh"), "w") as f:
    for i in range(256):
        f.write(f"harm5_lut[{i}] = 12'd{scaled(vals[i], 1, 10)};\n")

# 不平衡注入表（DIP2）：B 相 80%、C 相 60%，谐波同比例缩放，
# 保证各相 h3/基波 = 20%、h5/基波 = 10% 与幅值无关（THD 读数三相一致 300‰）。
# fund80 = 4/5，fund60 = 3/5，h3_80 = 4/25，h5_80 = 2/25，h3_60 = 3/25，h5_60 = 3/50。
unbal_tables = {
    "fund80_lut": (4, 5),
    "fund60_lut": (3, 5),
    "h3_80_lut":  (4, 25),
    "h5_80_lut":  (2, 25),
    "h3_60_lut":  (3, 25),
    "h5_60_lut":  (3, 50),
}
with open(os.path.join(output_dir, "unbal_lut_init.vh"), "w") as f:
    for name, (num, den) in unbal_tables.items():
        for i in range(256):
            f.write(f"{name}[{i}] = 12'd{scaled(vals[i], num, den)};\n")

print("Generated harm3_lut_init.vh (1/5), harm5_lut_init.vh (1/10), unbal_lut_init.vh (6 tables)")
