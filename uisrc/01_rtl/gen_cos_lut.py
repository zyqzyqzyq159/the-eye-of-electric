import math
vals = [round(math.cos(i*math.pi/256)*16384) for i in range(128)]
with open("cos_lut_init.vh", "w") as f:
    f.write("// cos 查找表（0..90°，Q14 有符号，128 点，步进 0.703°）
")
    for i, v in enumerate(vals):
        f.write(f"cos_lut[{i}] = 15'sd{v};
")
