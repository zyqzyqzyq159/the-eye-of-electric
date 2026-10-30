module dds_signal #(
    parameter CLK_FREQ       = 100_000_000,
    parameter DAC_BITS       = 12,
    parameter LUT_SIZE       = 256,
    parameter PHASE_ACC_BITS = 32
)(
    input  wire        clk,
    input  wire        rst_n,

    output wire [DAC_BITS-1:0] dds_va,
    output wire [DAC_BITS-1:0] dds_vb,
    output wire [DAC_BITS-1:0] dds_vc,
    // 三相电流自检源：与电压同频同幅、滞后 30°（模拟 cosφ=0.866 感性负载），
    // 谐波/不平衡/逆相序注入与电压联动（不平衡按 B 80%/C 60% 同比例缩放）
    output wire [DAC_BITS-1:0] dds_ia,
    output wire [DAC_BITS-1:0] dds_ib,
    output wire [DAC_BITS-1:0] dds_ic,
    output wire               dds_valid,

    input  wire [31:0] freq_config,
    input  wire [15:0] amp_config,

    input  wire        enable,
    input  wire        select,

    // 失真注入（自检算法验证）：1 = 基波叠加 20% 三次 + 10% 五次谐波，
    // 理论 THD = sqrt(0.2^2+0.1^2) = 22.4%（标准定义）/ 30%（本机求和定义）
    input  wire        distort_en,

    // 三相幅值不平衡注入（DIP2）：B 相 80%、C 相 60%（A 相满幅），
    // 谐波同比例缩放 → 各相 THD 读数一致；理论不平衡度 = 0.2/0.8 = 250‰
    input  wire        unbal_en,

    // 逆相序（DIP3）：B/C 输出互换 → 相序 ACB，相位差 +120°，seq=0
    input  wire        rev_seq
);

localparam PHASE_120 = 32'h55555555;
localparam PHASE_240 = 32'hAAAAAAAA;

reg [PHASE_ACC_BITS-1:0] phase_acc_a;
reg [PHASE_ACC_BITS-1:0] phase_acc_b;
reg [PHASE_ACC_BITS-1:0] phase_acc_c;

reg [PHASE_ACC_BITS-1:0] phase_incr;
reg [PHASE_ACC_BITS-1:0] phase_incr_shaded;

// 相位增量 = freq × 2^32 / CLK_FREQ。若直接截断 PHASE_INCR_PER_HZ（42.95 → 42），
// 50Hz 配置实际输出 48.88Hz，与 12.8kHz 采样失配产生频谱泄漏：
// RMS 偏高约 20%、纯正弦测出 ~10% 假 THD。改为保留 8bit 小数精度后四舍五入。
localparam [63:0] PHASE_INCR_PER_HZ_X256 = ((64'd1 << 40) + (CLK_FREQ >> 1)) / CLK_FREQ;
localparam [31:0] PHASE_INCR_PER_HZ_256L = PHASE_INCR_PER_HZ_X256[31:0];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        phase_incr <= 32'd0;
        phase_incr_shaded <= 32'd0;
    end else begin
        phase_incr_shaded <= (freq_config * PHASE_INCR_PER_HZ_256L + 32'd128) >> 8;
        phase_incr <= phase_incr_shaded;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 正序 ABC：B 滞后 A 120°（相位偏移 +240°），C 滞后 A 240°（偏移 +120°）
        phase_acc_a <= 32'd0;
        phase_acc_b <= PHASE_240;
        phase_acc_c <= PHASE_120;
    end else if (enable) begin
        phase_acc_a <= phase_acc_a + phase_incr;
        phase_acc_b <= phase_acc_b + phase_incr;
        phase_acc_c <= phase_acc_c + phase_incr;
    end
end

wire [7:0] lut_addr_a = phase_acc_a[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_b = phase_acc_b[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_c = phase_acc_c[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];

// 谐波相位 = 基波相位 ×3 / ×5（模 2^32 自然回绕；正弦周期 2π 对应 2^32，
// 相位乘 k 后查同一相位表即得 k 次谐波，无需独立累加器）
wire [PHASE_ACC_BITS-1:0] ph3_a = {phase_acc_a[30:0], 1'b0} + phase_acc_a;
wire [PHASE_ACC_BITS-1:0] ph3_b = {phase_acc_b[30:0], 1'b0} + phase_acc_b;
wire [PHASE_ACC_BITS-1:0] ph3_c = {phase_acc_c[30:0], 1'b0} + phase_acc_c;
wire [PHASE_ACC_BITS-1:0] ph5_a = {phase_acc_a[29:0], 2'b00} + phase_acc_a;
wire [PHASE_ACC_BITS-1:0] ph5_b = {phase_acc_b[29:0], 2'b00} + phase_acc_b;
wire [PHASE_ACC_BITS-1:0] ph5_c = {phase_acc_c[29:0], 2'b00} + phase_acc_c;

wire [7:0] lut_addr_a3 = ph3_a[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_b3 = ph3_b[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_c3 = ph3_c[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_a5 = ph5_a[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_b5 = ph5_b[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];
wire [7:0] lut_addr_c5 = ph5_c[PHASE_ACC_BITS-1:PHASE_ACC_BITS-8];

(* ram_style = "block" *) reg [DAC_BITS-1:0] sin_lut [0:LUT_SIZE-1];
// 谐波表：与 sin_lut 同相、幅度 1/5 与 1/10（gen_harm_lut.py 从 sin_lut_init.vh
// 派生，保证谐波与基波幅度成精确比例）
(* ram_style = "block" *) reg [DAC_BITS-1:0] harm3_lut [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] harm5_lut [0:LUT_SIZE-1];
// 不平衡表（B 相 80% / C 相 60%，基波与谐波同比例缩放，各相 THD 比例不变）
(* ram_style = "block" *) reg [DAC_BITS-1:0] fund80_lut [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] h3_80_lut   [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] h5_80_lut   [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] fund60_lut [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] h3_60_lut   [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] h5_60_lut   [0:LUT_SIZE-1];
// 电流自检表（gen_ilut.py 生成）：30° 滞后烘焙进表值，复用电压相位累加器地址，
// 无需独立电流累加器，也不需要双端口 ROM（每表仍单读口）
(* ram_style = "block" *) reg [DAC_BITS-1:0] isin_lut    [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] ih3_lut     [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] ih5_lut     [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] i80_fund_lut [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] i80_h3_lut   [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] i80_h5_lut   [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] i60_fund_lut [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] i60_h3_lut   [0:LUT_SIZE-1];
(* ram_style = "block" *) reg [DAC_BITS-1:0] i60_h5_lut   [0:LUT_SIZE-1];

initial begin
    `include "sin_lut_init.vh"
    `include "harm3_lut_init.vh"
    `include "harm5_lut_init.vh"
    `include "unbal_lut_init.vh"
    `include "dds_i_lut_init.vh"
end

reg [DAC_BITS-1:0] sin_val_a;
reg [DAC_BITS-1:0] sin_val_b;
reg [DAC_BITS-1:0] sin_val_c;
reg [DAC_BITS-1:0] h3_val_a, h3_val_b, h3_val_c;
reg [DAC_BITS-1:0] h5_val_a, h5_val_b, h5_val_c;
// B/C 相缩放读出（与满幅读出同地址同拍，unbal_en 在寄存器后选择）
reg [DAC_BITS-1:0] b80_fund, b80_h3, b80_h5;
reg [DAC_BITS-1:0] c60_fund, c60_h3, c60_h5;
// 电流读出（同地址同拍，30° 滞后已烘焙进表值）
reg [DAC_BITS-1:0] isin_val_a, isin_val_b, isin_val_c;
reg [DAC_BITS-1:0] ih3_val_a, ih3_val_b, ih3_val_c;
reg [DAC_BITS-1:0] ih5_val_a, ih5_val_b, ih5_val_c;
reg [DAC_BITS-1:0] ib80_fund, ib80_h3, ib80_h5;
reg [DAC_BITS-1:0] ic60_fund, ic60_h3, ic60_h5;

always @(posedge clk) begin
    sin_val_a <= sin_lut[lut_addr_a];
    sin_val_b <= sin_lut[lut_addr_b];
    sin_val_c <= sin_lut[lut_addr_c];
    h3_val_a  <= harm3_lut[lut_addr_a3];
    h3_val_b  <= harm3_lut[lut_addr_b3];
    h3_val_c  <= harm3_lut[lut_addr_c3];
    h5_val_a  <= harm5_lut[lut_addr_a5];
    h5_val_b  <= harm5_lut[lut_addr_b5];
    h5_val_c  <= harm5_lut[lut_addr_c5];
    b80_fund  <= fund80_lut[lut_addr_b];
    b80_h3    <= h3_80_lut[lut_addr_b3];
    b80_h5    <= h5_80_lut[lut_addr_b5];
    c60_fund  <= fund60_lut[lut_addr_c];
    c60_h3    <= h3_60_lut[lut_addr_c3];
    c60_h5    <= h5_60_lut[lut_addr_c5];
    isin_val_a <= isin_lut[lut_addr_a];
    isin_val_b <= isin_lut[lut_addr_b];
    isin_val_c <= isin_lut[lut_addr_c];
    ih3_val_a  <= ih3_lut[lut_addr_a3];
    ih3_val_b  <= ih3_lut[lut_addr_b3];
    ih3_val_c  <= ih3_lut[lut_addr_c3];
    ih5_val_a  <= ih5_lut[lut_addr_a5];
    ih5_val_b  <= ih5_lut[lut_addr_b5];
    ih5_val_c  <= ih5_lut[lut_addr_c5];
    ib80_fund  <= i80_fund_lut[lut_addr_b];
    ib80_h3    <= i80_h3_lut[lut_addr_b3];
    ib80_h5    <= i80_h5_lut[lut_addr_b5];
    ic60_fund  <= i60_fund_lut[lut_addr_c];
    ic60_h3    <= i60_h3_lut[lut_addr_c3];
    ic60_h5    <= i60_h5_lut[lut_addr_c5];
end

// 幅值选择流水级（时序修复）：原路径 BRAM clk2q(≈4ns) → unbal mux → 三表求和 →
// 饱和 → rev mux 单周期 11.2ns，违例 -1.3ns。插入一级寄存器切成两段：
// BRAM→mux→reg（≈8ns）与 reg→求和→饱和→rev mux→reg（≈8.5ns）。
// 三相统一加一拍：相间相位关系不变，整体延迟 10ns 对 50Hz 信号不可见
reg [DAC_BITS-1:0] fund_a1, h3_a1, h5_a1;
reg [DAC_BITS-1:0] fund_b1, h3_b1, h5_b1;
reg [DAC_BITS-1:0] fund_c1, h3_c1, h5_c1;
always @(posedge clk) begin
    fund_a1 <= sin_val_a;
    h3_a1   <= h3_val_a;
    h5_a1   <= h5_val_a;
    // unbal 时 B 用 80% 表、C 用 60% 表（mux 吸收进本拍寄存器）
    fund_b1 <= unbal_en ? b80_fund : sin_val_b;
    h3_b1   <= unbal_en ? b80_h3   : h3_val_b;
    h5_b1   <= unbal_en ? b80_h5   : h5_val_b;
    fund_c1 <= unbal_en ? c60_fund : sin_val_c;
    h3_c1   <= unbal_en ? c60_h3   : h3_val_c;
    h5_c1   <= unbal_en ? c60_h5   : h5_val_c;
end

// 电流幅值选择流水级（与电压同构：unbal 时 B/C 用 80%/60% 电流表，
// 谐波同比例缩放 → 各相 THD-I 读数一致）
reg [DAC_BITS-1:0] ifund_a1, ih3_a1, ih5_a1;
reg [DAC_BITS-1:0] ifund_b1, ih3_b1, ih5_b1;
reg [DAC_BITS-1:0] ifund_c1, ih3_c1, ih5_c1;
always @(posedge clk) begin
    ifund_a1 <= isin_val_a;
    ih3_a1   <= ih3_val_a;
    ih5_a1   <= ih5_val_a;
    ifund_b1 <= unbal_en ? ib80_fund : isin_val_b;
    ih3_b1   <= unbal_en ? ib80_h3   : ih3_val_b;
    ih5_b1   <= unbal_en ? ib80_h5   : ih5_val_b;
    ifund_c1 <= unbal_en ? ic60_fund : isin_val_c;
    ih3_c1   <= unbal_en ? ic60_h3   : ih3_val_c;
    ih5_c1   <= unbal_en ? ic60_h5   : ih5_val_c;
end

// 失真合成：v = 基波 + h3 + h5 − 2×2048（三表均 2048 中心）。
// 合成峰值 ≈ 0.9×满幅（θ≈90° 时 sin3θ 反相抵消），正常不触饱和；饱和为防御逻辑。
wire [13:0] dsum_a = {2'b00, fund_a1} + {2'b00, h3_a1} + {2'b00, h5_a1} - 14'd4096;
wire [13:0] dsum_b = {2'b00, fund_b1} + {2'b00, h3_b1} + {2'b00, h5_b1} - 14'd4096;
wire [13:0] dsum_c = {2'b00, fund_c1} + {2'b00, h3_c1} + {2'b00, h5_c1} - 14'd4096;

// 三表和 0..12185，减 4096 后真值 -4096..8089：14bit 回绕下 ≥12288 为负下溢，
// 4096..12287 为正上溢（真正值最大 8089，不会与负区重叠）
function [DAC_BITS-1:0] dist_sat;
    input [13:0] s;
    begin
        if      (s >= 14'd12288) dist_sat = {DAC_BITS{1'b0}};
        else if (s >  14'd4095)  dist_sat = {DAC_BITS{1'b1}};
        else                     dist_sat = s[DAC_BITS-1:0];
    end
endfunction

// 各相最终波形（失真合成或纯净基波，B/C 已含幅值选择）
wire [DAC_BITS-1:0] out_a = distort_en ? dist_sat(dsum_a) : fund_a1;
wire [DAC_BITS-1:0] out_b = distort_en ? dist_sat(dsum_b) : fund_b1;
wire [DAC_BITS-1:0] out_c = distort_en ? dist_sat(dsum_c) : fund_c1;

// 电流失真合成（与电压同构：20% h3 + 10% h5，30° 滞后在表内）
wire [13:0] idsum_a = {2'b00, ifund_a1} + {2'b00, ih3_a1} + {2'b00, ih5_a1} - 14'd4096;
wire [13:0] idsum_b = {2'b00, ifund_b1} + {2'b00, ih3_b1} + {2'b00, ih5_b1} - 14'd4096;
wire [13:0] idsum_c = {2'b00, ifund_c1} + {2'b00, ih3_c1} + {2'b00, ih5_c1} - 14'd4096;

wire [DAC_BITS-1:0] iout_a = distort_en ? dist_sat(idsum_a) : ifund_a1;
wire [DAC_BITS-1:0] iout_b = distort_en ? dist_sat(idsum_b) : ifund_b1;
wire [DAC_BITS-1:0] iout_c = distort_en ? dist_sat(idsum_c) : ifund_c1;

reg [DAC_BITS-1:0] dds_va_reg;
reg [DAC_BITS-1:0] dds_vb_reg;
reg [DAC_BITS-1:0] dds_vc_reg;
reg [DAC_BITS-1:0] dds_ia_reg;
reg [DAC_BITS-1:0] dds_ib_reg;
reg [DAC_BITS-1:0] dds_ic_reg;
reg                dds_valid_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dds_va_reg    <= 12'd0;
        dds_vb_reg    <= 12'd0;
        dds_vc_reg    <= 12'd0;
        dds_ia_reg    <= 12'd0;
        dds_ib_reg    <= 12'd0;
        dds_ic_reg    <= 12'd0;
        dds_valid_reg <= 1'b0;
    end else begin
        if (enable && select) begin
            dds_va_reg    <= out_a;
            dds_vb_reg    <= rev_seq ? out_c : out_b;   // 逆相序：B/C 互换
            dds_vc_reg    <= rev_seq ? out_b : out_c;
            dds_ia_reg    <= iout_a;
            dds_ib_reg    <= rev_seq ? iout_c : iout_b; // 电流 B/C 同步互换
            dds_ic_reg    <= rev_seq ? iout_b : iout_c;
            dds_valid_reg <= 1'b1;
        end else begin
            dds_valid_reg <= 1'b0;
        end
    end
end

assign dds_va    = dds_va_reg;
assign dds_vb    = dds_vb_reg;
assign dds_vc    = dds_vc_reg;
assign dds_ia    = dds_ia_reg;
assign dds_ib    = dds_ib_reg;
assign dds_ic    = dds_ic_reg;
assign dds_valid = dds_valid_reg;

endmodule
