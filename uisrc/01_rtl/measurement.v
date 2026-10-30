// 测量模块：6 通道（VA/VB/VC/IA/IB/IC）RMS / THD / 谐波百分比 / 相位差 / 不平衡度
// 单位约定：THD、谐波百分比、不平衡度为千分比；电压 RMS 为 0.1V，电流 RMS 为 0.1A；
// 相位 16bit：65536 = 360°；频率 = 基波 bin × 50Hz
//
// 标定（采样板信号链，见 AL.md；相干满幅仿真 calib_rms_tb 实测 rms_code=2065，
// 即 FFT 链路对相干正弦输出 ≈ 峰值码数 × 1.0088，常数已含该补偿；
// pll_sync 重采样链实测再低 0.6%，常数按链路实测校准使读数=物理真值）：
//   电压（实际采样板）：逆变器输出 → 2k/30k 分压(1/16) → AMC1350(0.4V/V，
//     差分输出限幅 ±1.25V) → 差分转单端(×1, 1.25V 偏置) → AD7928(2.5V 量程)
//     一次侧每 code = (2.5/4096)×40 = 24.414mV（1/16 × 0.4 × 1 = 1/40）
//     满幅 ±1.25V ↔ 一次侧 ±50V 峰 = 35.36V RMS
//     RMS_MUL_V = 2828 → 满幅读数 354（35.4V，= 50/√2，实测校准）
//   电流：CC6905S8-3FB020(±20A,66mV/A) → 差分级 G=0.947(十.4 设计) → AD7928
//     每 code = 20000mA/2047 = 9.770mA；满幅 ±1.25V ↔ ±20.00A 峰 = 14.14A RMS
//     RMS_MUL_I = 1129 → 满幅读数 141（14.1A，= 20/√2，实测校准）
//     显示 ±20A 挡半窗码数同步改 2047，自检波形峰恰好贴窗顶刻度标签
//   rms_code_w = 信号峰值 code 数（FFT 各 bin 幅度和 ≈ 峰值×128，>>7 还原），故乘 1/√2
//
// 除法采用 32 拍恢复余数法时序除法器（谐波百分比 45 次 + 不平衡度 1 次/轮）
// 分子装载 3 拍流水：RAM读 → ×1000 → 装载（原单拍 RAM读+乘法 13.81ns 超 100MHz 周期 3.81ns）
// 基波查找 2 拍流水：FIND_FUND 仅读 RAM，FIND_ACC 比较基波 + RMS 累加
//（原单拍 读+比较+累加 11.05ns 超 100MHz 周期 1.4ns）
module measurement #(
    parameter FFT_SIZE = 256,
    parameter HARMONICS_NUM = 15,
    parameter RMS_MUL_V = 2828,     // 电压：2k/30k 分压 1/16 × AMC1350 0.4 × 差放 1，0.1V 显示（满幅 354）
    parameter RMS_MUL_I = 1129,     // 电流：CC6905 ±20A × G=0.947，0.1A 显示（满幅 141）
    parameter RMS_SHIFT = 14
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0]         mag_in,
    input  wire [15:0]         phase_in,
    input  wire [7:0]          bin_in,
    input  wire                mag_valid,
    input  wire                fft_done,
    input  wire [2:0]          fft_channel,

    output wire [31:0] fundamental_freq,
    output wire [31:0] rms_va,
    output wire [31:0] rms_vb,
    output wire [31:0] rms_vc,
    output wire [31:0] rms_ia,
    output wire [31:0] rms_ib,
    output wire [31:0] rms_ic,
    output wire [31:0] thd_va,
    output wire [31:0] thd_vb,
    output wire [31:0] thd_vc,
    output wire [31:0] thd_ia,
    output wire [31:0] thd_ib,
    output wire [31:0] thd_ic,
    output wire [HARMONICS_NUM*16-1:0] harm_pct_va,
    output wire [HARMONICS_NUM*16-1:0] harm_pct_vb,
    output wire [HARMONICS_NUM*16-1:0] harm_pct_vc,
    output wire [HARMONICS_NUM*16-1:0] harm_pct_ia,
    output wire [HARMONICS_NUM*16-1:0] harm_pct_ib,
    output wire [HARMONICS_NUM*16-1:0] harm_pct_ic,
    output wire [15:0] phase_ab,
    output wire [15:0] phase_bc,
    output wire [15:0] phase_ca,
    output wire [15:0] phase_ia,    // A 相 V→I 相位角（电流滞后为负）
    output wire [15:0] phase_ib,
    output wire [15:0] phase_ic,
    output wire        phase_sequence,
    output wire [31:0] unbalance_degree,
    // 功率/功率因数/补偿建议总线（每轮测量更新，与 done_toggle 同步有效）
    // {pf_a/pf_b/pf_c Q14, pa/pb/pc W, qa/qb/qc |var|, ptot W, qtot var,
    //  cap_uf 补偿电容 µF, l_01mh 补偿电感 0.1mH, cur_pres, pf_ok, lag}
    output wire [177:0] pwr_bus,
    output wire        measure_done,
    output wire        done_toggle,     // 每轮测量完成翻转一次（跨时钟域握手）
    input  wire        enable
);

localparam HALF_FFT = FFT_SIZE / 2;

reg [31:0] rms_reg [0:5];
// 不平衡度专用高精度 RMS（仅电压通道 0..2）：0.1V×16 单位（rms_prod 右移 10）。
// 0.1V 整数量化（12/9/7 vs 真值 11.6/9.3/7.0）使 100/80/60% 注入读出 333‰ 而非 250‰
reg [17:0] rms_fine [0:5];
reg [31:0] thd_reg [0:5];
reg [15:0] harm_pct_reg [0:5][0:HARMONICS_NUM-1];
reg [31:0] fund_freq_reg;
reg [15:0] phase_ab_reg, phase_bc_reg, phase_ca_reg;
reg [15:0] phase_ia_reg, phase_ib_reg, phase_ic_reg;
reg        phase_seq_reg;
reg [31:0] unbalance_reg;

reg [4:0] state;
localparam IDLE         = 5'd0;
localparam CAPTURE_MAG  = 5'd1;
localparam FIND_FUND    = 5'd2;
localparam FIND_ACC     = 5'd3;    // FIND_FUND 拍2：基波比较 + RMS 累加
localparam DIV_SETUP    = 5'd4;
localparam DIV_RUN      = 5'd5;
localparam STORE_PCT    = 5'd6;
localparam CALC_PHASE   = 5'd7;
localparam CALC_UNBAL   = 5'd8;
localparam UNB_STORE    = 5'd9;
localparam DONE         = 5'd10;
localparam UNB_MUL      = 5'd11;
localparam UNB_DEV      = 5'd12;
localparam UNB_MAX      = 5'd13;
localparam DIV_RD       = 5'd14;   // 分子拍1：谐波幅度 RAM 读取
localparam DIV_MUL      = 5'd15;   // 分子拍2：分子 ×1000
localparam DIV_SAT      = 5'd16;   // 除法完成拍：div_q 饱和到 1023 寄存
localparam UNB_MAX2     = 5'd17;   // 取最大拍2：dev2 与 max01_r 比较
localparam PW_COS       = 5'd18;   // 功率：取 φ 的 cos/sin + V×I 乘法（每相 3 拍流水）
localparam PW_ABS       = 5'd23;   // 功率：|cos|/|sin| 寄存（取负与宽乘法串拍曾致 100MHz -3.6ns）
localparam PW_MUL       = 5'd19;   // 功率：(V×I)×cosφ / ×sinφ 乘法
localparam PW_ST        = 5'd20;   // 功率：pm/qm 乘积寄存（V19b 拆拍）
localparam PW_ST2       = 5'd25;   // 功率：P/Q 存储饱和，下一相或合计（V19b）
localparam PW_TOT       = 5'd21;   // 功率：三相合计 + 电流检测 + PF 判定 + 补偿除法装载
localparam COMP_FIN     = 5'd22;   // 补偿容量计算完成（C µF 或 L 0.1mH）
localparam COMP_MUL     = 5'd24;   // 补偿容量乘法拍（×常数与宽比较串拍曾致 100MHz -3ns）

reg [2:0]  cur_phase;
reg [7:0]  cap_cnt;
reg [7:0]  fund_bin;
reg [7:0]  fund_bin_a;
reg [31:0] fund_mag;
reg [31:0] rms_sum;
reg [31:0] thd_sum;
reg [3:0]  div_idx;
reg [7:0]  harm_addr;

localparam [7:0] MAX_BIN = 8'd32;   // 只存 bin0..31：15 次谐波（bin15@750Hz）以上被前端抗混叠滤波抑制
                                    // LUT-RAM 成本 ∝ 深度，128→32 省约 3000 LUT（曾致 LUT 114% 超标）
(* ram_style = "block" *) reg [31:0] mag_buf [0:5][0:MAX_BIN-1];
(* ram_style = "block" *) reg [15:0] phase_buf [0:5][0:MAX_BIN-1];

reg measure_done_reg;
reg done_toggle_reg;

// ---- 功率/PF/补偿寄存器 ----
reg [1:0]  pw_i;                    // 功率计算相序 0..2
reg signed [14:0] pw_cos_r;         // 当前相 cosφ (Q14)
reg signed [14:0] pw_sin_r;         // 当前相 sinφ (Q14)
reg               pw_cs_r;          // 当前相 cosφ 符号（|φ|>90° 时 P 显示 0）
reg [14:0] pw_cos_abs_r;          // |cosφ| 寄存（PW_ABS 拍，切断 取负→宽乘法 同拍路径）
reg [14:0] pw_sin_abs_r;            // |sinφ| 寄存
wire [14:0] pw_cos_abs = pw_cos_r[14] ? -pw_cos_r : pw_cos_r;
wire [14:0] pw_sin_abs = pw_sin_r[14] ? -pw_sin_r : pw_sin_r;
wire        cos16_neg  = cos16(pw_phi_sel) < 0;   // 函数结果先比较再取位（位选函数调用非法）
reg [31:0] pw_vi_r;                 // V_01 × I_01
reg [53:0] pw_pm_r, pw_qm_r;        // (V×I)×cosφ / ×sinφ（Q14，宽乘积）
reg [59:0] pw_pwx_r, pw_qwx_r;      // ×41>>26 乘积寄存（V19b：54bit 乘法占 fabric，
                                    // 拆一拍寄存 + ×41 改移位加，省 LUT 缓布线拥塞）
reg [15:0] pw_p [0:2];              // 有功 W
reg [15:0] pw_q [0:2];              // 无功 |var|
reg [14:0] pw_pfa_r;                // A 相 PF（Q14，无符号）
reg [15:0] pw_ptot, pw_qtot;        // 三相合计
reg [31:0] pw_vsq_r;                // A 相 V²（补偿除法用）
reg [31:0] pw_comp_den;             // 补偿除法分母
reg [15:0] pw_cap_r, pw_ind_r;      // 补偿电容 µF / 电感 0.1mH
reg [32:0] pw_capm_r, pw_indm_r;    // 补偿容量乘积寄存（COMP_MUL 拍，切断 乘法→宽比较 同拍路径）
reg        pw_cpres_r, pw_pfok_r, pw_lag_r;

wire [177:0] pwr_bus_w = {pw_pfa_r, pw_p[0], pw_p[1], pw_p[2],
                          pw_q[0], pw_q[1], pw_q[2], pw_ptot, pw_qtot,
                          pw_cap_r, pw_ind_r, pw_cpres_r, pw_pfok_r, pw_lag_r};
assign pwr_bus = pwr_bus_w;
// 当前相 V→I 相位角选择
wire [15:0] pw_phi_sel = (pw_i == 2'd0) ? phase_ia_reg :
                         (pw_i == 2'd1) ? phase_ib_reg : phase_ic_reg;
// P/W 与 Q/var 数值（(V×I)×cosφ×41>>26 ≈ /100.07；负值截 0，饱和 65535）
// V19b：×41 用移位加实现（41=32+8+1），替代 54bit×6 乘法（DSP 已满、乘法
// 溢出到 fabric 加重拥塞——medium effort 下 PHY-8023 的成因之一）
wire [59:0] pw_pwx = ((pw_pwx_r << 5) + (pw_pwx_r << 3) + pw_pwx_r) >> 26;
wire [59:0] pw_qwx = ((pw_qwx_r << 5) + (pw_qwx_r << 3) + pw_qwx_r) >> 26;
wire [31:0] pw_comp_den_x = {16'd0, pw_q[0]};

// 不平衡度流水线寄存器（5 拍：求和 → ×1/3 取平均 → 三路偏差 → max(dev0,dev1) → max(·,dev2)）
// 原单拍 36 级组合逻辑 25.3ns，严重超 100MHz 周期；
// 两级 max 串联（22 级加法器链 10.2ns）仍违例 -0.497ns，故再拆一拍
reg [33:0] rms3_r, dev0_r, dev1_r, dev2_r;
reg [49:0] avg_mul_r;
reg [31:0] unb_num_r;
reg [33:0] max01_r;
wire [33:0] avg_w   = (avg_mul_r + 50'd32768) >> 16;   // 四舍五入防平衡时截断出假偏差
wire [33:0] max01_w = (dev0_r > dev1_r) ? dev0_r : dev1_r;
wire [33:0] max_d_w = (dev2_r > max01_r) ? dev2_r : max01_r;

// 时序除法器：div_q = div_num / div_den（恢复余数法，32 拍）
reg [31:0] div_rem, div_q, div_den;
reg [5:0]  div_cnt;
reg [1:0]  div_task;   // 0: 谐波百分比  1: 不平衡度  2: 补偿容量
reg [31:0] div_sat_r;  // div_q 饱和到 1023 的寄存值（DIV_SAT 拍锁存）

// 分子 3 拍装载流水寄存器（RAM 读出 → ×1000 → 装载到除法器）
reg [31:0] mag_rd_r;
reg [15:0] ph0_r, ph1_r, ph2_r;    // 相序判定用 phase_buf 寄存读（消除异步读 → 全 RAM 可映射 ERAM）

// cos 查找表 0..90° Q14（cos_lut_init.vh 由 gen_cos_lut.py 生成）
(* rom_style = "block" *) reg signed [14:0] cos_lut [0:127];
initial begin
    `include "cos_lut_init.vh"
end

// 16bit 角度（65536=360°）→ cos Q14：四象限折叠，精度 ±0.7°（PF 误差 <0.012）
function signed [14:0] cos16;
    input [15:0] a;
    reg [1:0] quad;
    reg [6:0] t;
    begin
        quad = a[15:14];
        t    = a[13:7];
        case (quad)
            2'd0: cos16 =  cos_lut[t];
            2'd1: cos16 = -cos_lut[7'd127 - t];
            2'd2: cos16 = -cos_lut[t];
            default: cos16 =  cos_lut[7'd127 - t];
        endcase
    end
endfunction
// sin(x) = cos(x - 90°)
function signed [14:0] sin16;
    input [15:0] a;
    begin
        sin16 = cos16(a - 16'd16384);
    end
endfunction
reg [31:0] div_num_r;
wire [31:0] div_mul_in = (div_task == 1'b0) ? mag_rd_r : unb_num_r;

wire [32:0] div_shift = {div_rem[30:0], div_q[31]};
wire        div_ge     = (div_shift >= {1'b0, div_den});


// STORE_PCT 取数用已寄存的饱和值 div_sat_r（原 饱和+累加+饱和 单拍 10.15ns 超 10ns 周期）
wire [31:0] thd_new_w = thd_sum + ((div_idx != 4'd0) ? div_sat_r : 32'd0);

// 电压/电流通道标定选择（cur_phase 在整段除法链期间稳定）
wire [15:0] rms_mul_sel = (cur_phase >= 3'd3) ? RMS_MUL_I[15:0] : RMS_MUL_V[15:0];

// RMS 码值 -> 0.1V/0.1A：+0.5LSB 四舍五入（否则 1.17V 截断成 1.1），
// 电压饱和 999.9V、电流满量程 13.4A 不触饱和
// 3 拍流水：两个 13×16 小乘法（各 fit 单个 MULT18）→ 拼接加法 → 舍入。
// （原 25×16 单拍大乘法：DSP+部分积加法树+布线 12.5ns 超 10ns 周期；
//   rms_sum 在 STORE_PCT 取数前至少 35 拍未变，流水延迟不影响结果）
wire [24:0] rms_code_w = rms_sum >> 7;
// V19：乘法操作数先寄存一拍——rms_sum 扇出 54，原 rms_sum→移位→MULT18
// 单拍 11.0ns（净延迟占 3.9ns）100MHz 违例 -1.2ns，实机偶发坏 RMS/P/Q/COMP
// 读数（PF 显示 38.8 即其后果）。rms_sum 在取数前 ≥35 拍未变，+1 拍无影响
reg  [24:0] rms_c_r;
reg  [15:0] rms_m_r;
reg  [28:0] rms_lo_r, rms_hi_r;
reg  [47:0] rms_prod_r, rms_rnd_r;
always @(posedge clk) begin
    rms_c_r    <= rms_code_w;
    rms_m_r    <= rms_mul_sel;
    rms_lo_r   <= rms_c_r[12:0]  * rms_m_r;
    rms_hi_r   <= rms_c_r[24:13] * rms_m_r;
    rms_prod_r <= ({19'd0, rms_hi_r} << 13) + {19'd0, rms_lo_r};
    rms_rnd_r  <= rms_prod_r + (48'd1 << (RMS_SHIFT-1));
end
wire [31:0] rms_dec_w   = (rms_rnd_r[47:RMS_SHIFT] > 32'd9999) ? 32'd9999
                                                               : rms_rnd_r[RMS_SHIFT+31:RMS_SHIFT];
// 高精度 RMS（0.1V×16，仅电压通道参与不平衡度）：饱和 999.9V×16 = 159984
wire [37:0] rms_fine_w  = rms_prod_r >> 10;

// 基波为零（无信号）时跳过除法，谐波/THD 直接清零
integer k;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= IDLE;
        cur_phase       <= 3'd0;
        cap_cnt         <= 8'd0;
        fund_bin        <= 8'd0;
        fund_bin_a      <= 8'd1;
        fund_mag        <= 32'd0;
        rms_sum         <= 32'd0;
        thd_sum         <= 32'd0;
        div_idx         <= 4'd0;
        harm_addr       <= 8'd0;
        measure_done_reg <= 1'b0;
        done_toggle_reg <= 1'b0;
        fund_freq_reg   <= 32'd0;
        phase_ab_reg    <= 16'd0;
        phase_bc_reg    <= 16'd0;
        phase_ca_reg    <= 16'd0;
        phase_ia_reg    <= 16'd0;
        phase_ib_reg    <= 16'd0;
        phase_ic_reg    <= 16'd0;
        phase_seq_reg   <= 1'b0;
        unbalance_reg   <= 32'd0;
        div_rem         <= 32'd0;
        div_q           <= 32'd0;
        div_den         <= 32'd1;
        div_cnt         <= 6'd0;
        div_task        <= 1'b0;
        div_sat_r       <= 32'd0;
        rms3_r          <= 34'd0;
        dev0_r          <= 34'd0;
        dev1_r          <= 34'd0;
        dev2_r          <= 34'd0;
        avg_mul_r       <= 50'd0;
        unb_num_r       <= 32'd0;
        max01_r         <= 34'd0;
        mag_rd_r        <= 32'd0;
        div_num_r       <= 32'd0;
        pw_i            <= 2'd0;
        pw_cos_r        <= 15'sd0;
        pw_cs_r         <= 1'b0;
        pw_sin_r        <= 15'sd0;
        pw_cos_abs_r    <= 15'd0;
        pw_sin_abs_r    <= 15'd0;
        pw_vi_r         <= 32'd0;
        pw_pwx_r        <= 60'd0;
        pw_qwx_r        <= 60'd0;
        pw_pm_r         <= 54'd0;
        pw_qm_r         <= 54'd0;
        pw_pfa_r        <= 15'd0;
        pw_ptot         <= 16'd0;
        pw_qtot         <= 16'd0;
        pw_vsq_r        <= 32'd0;
        pw_comp_den     <= 32'd1;
        pw_cap_r        <= 16'd0;
        pw_ind_r        <= 16'd0;
        pw_capm_r       <= 33'd0;
        pw_indm_r       <= 33'd0;
        pw_cpres_r      <= 1'b0;
        pw_pfok_r       <= 1'b0;
        pw_lag_r        <= 1'b0;
        pw_p[0] <= 16'd0; pw_p[1] <= 16'd0; pw_p[2] <= 16'd0;
        pw_q[0] <= 16'd0; pw_q[1] <= 16'd0; pw_q[2] <= 16'd0;
    end else begin
        case (state)
            IDLE: begin
                measure_done_reg <= 1'b0;
                if (mag_valid && enable) begin
                    cur_phase <= fft_channel;
                    if (bin_in < MAX_BIN) begin
                        mag_buf[fft_channel][bin_in] <= mag_in;
                        phase_buf[fft_channel][bin_in] <= phase_in;
                    end
                    state <= CAPTURE_MAG;
                end
            end

            CAPTURE_MAG: begin
                if (mag_valid) begin
                    if (bin_in < MAX_BIN) begin
                        mag_buf[fft_channel][bin_in] <= mag_in;
                        phase_buf[fft_channel][bin_in] <= phase_in;
                    end
                end
                if (fft_done) begin
                    fund_bin <= 8'd1;
                    fund_mag <= mag_buf[cur_phase][1];
                    rms_sum <= mag_buf[cur_phase][1];   // RMS 从 bin1 起累加（剔除直流）
                    cap_cnt <= 8'd2;
                    state   <= FIND_FUND;
                end
            end

            FIND_FUND: begin                      // 拍1：仅 RAM 读取（原单拍 读+比较+累加 11.05ns 超 10ns 周期）
                if (cap_cnt < MAX_BIN) begin
                    mag_rd_r <= mag_buf[cur_phase][cap_cnt];
                    cap_cnt  <= cap_cnt + 1'b1;
                    state    <= FIND_ACC;
                end else if (fund_mag == 32'd0) begin
                    // 无信号：谐波/THD 清零，跳过全部除法
                    for (k = 0; k < HARMONICS_NUM; k = k + 1)
                        harm_pct_reg[cur_phase][k] <= 16'd0;
                    thd_reg[cur_phase] <= 32'd0;
                    rms_reg[cur_phase] <= 32'd0;
                    if (cur_phase < 3'd5) begin
                        cur_phase <= cur_phase + 3'd1;
                        fund_bin  <= 8'd1;
                        fund_mag  <= mag_buf[cur_phase+1][1];
                        rms_sum   <= mag_buf[cur_phase+1][1];
                        cap_cnt   <= 8'd2;
                        state     <= FIND_FUND;
                    end else begin
                        state <= CALC_PHASE;
                    end
                end else begin
                    if (cur_phase == 3'd0) begin
                        fund_bin_a  <= fund_bin;
                        fund_freq_reg <= {fund_bin, 5'd0} + {fund_bin, 4'd0} + {fund_bin, 1'd0};  // ×50Hz
                    end
                    harm_addr <= fund_bin;
                    div_idx   <= 4'd0;
                    thd_sum   <= 32'd0;
                    state     <= DIV_RD;
                end
            end

            FIND_ACC: begin                       // 拍2：基波比较 + RMS 累加
                if (mag_rd_r > fund_mag) begin
                    fund_mag <= mag_rd_r;
                    fund_bin <= cap_cnt - 8'd1;   // 拍1已自增，回退得本拍数据对应 bin
                end
                rms_sum <= rms_sum + mag_rd_r;
                state   <= FIND_FUND;
            end

            DIV_RD: begin
                // 谐波地址钳位：fund_bin=1 时 harm_addr=k≤15；防瞬态下 fund_bin 偏大越界
                mag_rd_r <= mag_buf[cur_phase][harm_addr[4:0]];
                state    <= DIV_MUL;
            end

            DIV_MUL: begin
                div_num_r <= (div_mul_in * 32'd125) << 3;   // ×1000（125<<3，结果与×1000逐位一致）
                state     <= DIV_SETUP;
            end

            DIV_SETUP: begin
                div_den <= (div_task == 2'd0) ? fund_mag :
                           (div_task == 2'd1) ? avg_w[31:0] : pw_comp_den;
                {div_rem, div_q} <= {32'd0, div_num_r};
                div_cnt <= 6'd32;
                state   <= DIV_RUN;
            end

            DIV_RUN: begin
                if (div_ge) begin
                    div_rem <= div_shift - {1'b0, div_den};
                    div_q   <= {div_q[30:0], 1'b1};
                end else begin
                    div_rem <= div_shift[31:0];
                    div_q   <= {div_q[30:0], 1'b0};
                end
                if (div_cnt == 6'd1)
                    state <= DIV_SAT;
                else
                    div_cnt <= div_cnt - 1'b1;
            end

            DIV_SAT: begin                          // div_q 饱和寄存一拍，切断 STORE_PCT 长链
                div_sat_r <= (div_q > 32'd1023) ? 32'd1023 : div_q;
                state     <= (div_task == 2'd0) ? STORE_PCT :
                             (div_task == 2'd1) ? UNB_STORE : COMP_MUL;
            end

            STORE_PCT: begin
                harm_pct_reg[cur_phase][div_idx] <= div_sat_r[15:0];
                if (div_idx == 4'd14) begin
                    thd_reg[cur_phase] <= (thd_new_w > 32'd9999) ? 32'd9999 : thd_new_w;
                    rms_reg[cur_phase] <= rms_dec_w;
                    rms_fine[cur_phase] <= (rms_fine_w > 38'd159984) ? 18'd159984
                                                                     : rms_fine_w[17:0];
                    ph0_r <= phase_buf[0][fund_bin_a];
                    ph1_r <= phase_buf[1][fund_bin_a];
                    ph2_r <= phase_buf[2][fund_bin_a];
                    if (cur_phase < 3'd5) begin
                        cur_phase <= cur_phase + 1'b1;
                        fund_bin  <= 8'd1;
                        fund_mag  <= mag_buf[cur_phase+1][1];
                        rms_sum   <= mag_buf[cur_phase+1][1];
                        cap_cnt   <= 8'd2;
                        state     <= FIND_FUND;
                    end else begin
                        state <= CALC_PHASE;
                    end
                end else begin
                    if (div_idx != 4'd0)
                        thd_sum <= thd_sum + ((div_q > 32'd1023) ? 32'd1023 : div_q);
                    div_idx   <= div_idx + 1'b1;
                    harm_addr <= harm_addr + fund_bin;
                    state <= DIV_RD;
                end
            end

            CALC_PHASE: begin
                phase_ab_reg <= phase_buf[1][fund_bin_a] - phase_buf[0][fund_bin_a];
                phase_bc_reg <= phase_buf[2][fund_bin_a] - phase_buf[1][fund_bin_a];
                phase_ca_reg <= phase_buf[0][fund_bin_a] - phase_buf[2][fund_bin_a];
                // V→I 相位角：各相电流基波相位减电压基波相位（感性负载为负）
                phase_ia_reg <= phase_buf[3][fund_bin_a] - phase_buf[0][fund_bin_a];
                phase_ib_reg <= phase_buf[4][fund_bin_a] - phase_buf[1][fund_bin_a];
                phase_ic_reg <= phase_buf[5][fund_bin_a] - phase_buf[2][fund_bin_a];
                // 相序：B 滞后 A 约 120°（±45°容差）判定正序 ABC（用寄存读，无异步 RAM 访问）
                phase_seq_reg <= ((ph1_r - ph0_r) >= -16'sd30037) &&
                                 ((ph1_r - ph0_r) <= -16'sd13653) &&
                                 ((ph2_r - ph1_r) >= -16'sd30037) &&
                                 ((ph2_r - ph1_r) <= -16'sd13653);
                state <= CALC_UNBAL;
            end

            // 不平衡度 5 拍流水：每拍逻辑 < 9ns，满足 100MHz
            // 用高精度 rms_fine（0.1V×16）计算：偏差最大 159984，×1000 = 1.6e8 < 2^31
            // 仅电压通道 0..2 参与
            CALC_UNBAL: begin
                rms3_r <= {16'd0, rms_fine[0]} + {16'd0, rms_fine[1]} + {16'd0, rms_fine[2]};
                state  <= UNB_MUL;
            end

            UNB_MUL: begin
                avg_mul_r <= rms3_r * 16'd21845;         // ×(1/3)：21845/65536
                state     <= UNB_DEV;
            end

            UNB_DEV: begin
                if (avg_w == 34'd0) begin
                    unbalance_reg <= 32'd0;
                    state <= DONE;
                end else begin
                    dev0_r <= (rms_fine[0] > avg_w) ? (rms_fine[0] - avg_w) : (avg_w - rms_fine[0]);
                    dev1_r <= (rms_fine[1] > avg_w) ? (rms_fine[1] - avg_w) : (avg_w - rms_fine[1]);
                    dev2_r <= (rms_fine[2] > avg_w) ? (rms_fine[2] - avg_w) : (avg_w - rms_fine[2]);
                    state  <= UNB_MAX;
                end
            end

            UNB_MAX: begin                          // 取最大拍1：仅 max(dev0,dev1)，切断两级比较器串联
                max01_r <= max01_w;
                state   <= UNB_MAX2;
            end

            UNB_MAX2: begin                         // 取最大拍2：与 dev2 比较 + 判零
                unb_num_r <= max_d_w[31:0];
                if (max_d_w == 34'd0) begin
                    unbalance_reg <= 32'd0;
                    state <= DONE;
                end else begin
                    div_task <= 1'b1;
                    state    <= DIV_MUL;
                end
            end

            UNB_STORE: begin
                unbalance_reg <= div_sat_r;
                div_task <= 2'd0;
                pw_i      <= 2'd0;
                state     <= PW_COS;      // → 功率/PF/补偿计算
            end

            // ---- 功率/PF/补偿：每相 4 拍流水（cos/sin → |abs|寄存 → 乘法 → 存储），A 相另算补偿 ----
            PW_COS: begin
                pw_cos_r <= cos16(pw_phi_sel);
                pw_sin_r <= sin16(pw_phi_sel);
                pw_cs_r  <= cos16_neg;               // cosφ < 0（|φ|>90°）
                // V19b：rms 值饱和上限 9999（14bit），按真实位宽乘（原 32×32
                // 全宽乘法在 DSP 满载后溢到 fabric，加重布线拥塞）
                pw_vi_r  <= rms_reg[pw_i][13:0] * rms_reg[3'd3 + {1'b0, pw_i}][13:0];
                state    <= PW_ABS;
            end

            PW_ABS: begin
                pw_cos_abs_r <= pw_cos_abs;
                pw_sin_abs_r <= pw_sin_abs;
                state        <= PW_MUL;
            end

            PW_MUL: begin
                pw_pm_r <= pw_vi_r[26:0] * {1'b0, pw_cos_abs_r};   // (V×I)×|cosφ|（27×15，V19b 收窄）
                pw_qm_r <= pw_vi_r[26:0] * {1'b0, pw_sin_abs_r};   // (V×I)×|sinφ|
                state   <= PW_ST;
            end

            PW_ST: begin
                pw_pwx_r <= pw_pm_r;
                pw_qwx_r <= pw_qm_r;
                state    <= PW_ST2;
            end

            PW_ST2: begin
                pw_p[pw_i] <= pw_cs_r ? 16'd0 :                        // |φ|>90°：P 显示 0
                              (pw_pwx > 60'd65535) ? 16'd65535 : pw_pwx[15:0];
                pw_q[pw_i] <= (pw_qwx > 60'd65535) ? 16'd65535 : pw_qwx[15:0];
                if (pw_i == 2'd0) pw_pfa_r <= pw_cos_r[14] ? -pw_cos_r : pw_cos_r;  // |cosφ|
                if (pw_i == 2'd2) begin
                    pw_i  <= 2'd0;
                    state <= PW_TOT;
                end else begin
                    pw_i   <= pw_i + 2'd1;
                    state  <= PW_COS;
                end
            end

            PW_TOT: begin
                pw_ptot <= ({17'd0, pw_p[0]} + {17'd0, pw_p[1]} + {17'd0, pw_p[2]} > 49'd65535)
                           ? 16'd65535 : pw_p[0] + pw_p[1] + pw_p[2];
                pw_qtot <= ({17'd0, pw_q[0]} + {17'd0, pw_q[1]} + {17'd0, pw_q[2]} > 49'd65535)
                           ? 16'd65535 : pw_q[0] + pw_q[1] + pw_q[2];
                pw_cpres_r <= (rms_reg[3] > 32'd5) || (rms_reg[4] > 32'd5) || (rms_reg[5] > 32'd5);
                pw_pfok_r  <= (pw_pfa_r > 15'sd16379);           // PF > 0.9996
                pw_lag_r   <= phase_ia_reg[15];                  // φA 符号位：1=负=电流滞后=感性
                pw_vsq_r   <= rms_reg[0][13:0] * rms_reg[0][13:0]; // A 相 V²（V19b 14×14 收窄）
                if (pw_cpres_r && !pw_pfok_r && rms_reg[0] > 32'd0) begin
                    // 需补偿：感性→并联电容 C=Q/(ωV²)；容性→串联电感 L=V²/(ωQ)
                    // 复用时序除法器（task=2），分子 ×16384 定标提高小商精度
                    div_task  <= 2'd2;
                    if (pw_lag_r) begin
                        div_num_r   <= {16'd0, pw_q[0]} << 14;    // Q × 16384
                        pw_comp_den <= pw_vsq_r;                  // V²
                    end else begin
                        div_num_r   <= pw_vsq_r;                  // V²
                        pw_comp_den <= (pw_q[0] == 16'd0) ? 32'd1 : pw_comp_den_x;
                    end
                    state <= DIV_SETUP;
                end else begin
                    pw_cap_r <= 16'd0;
                    pw_ind_r <= 16'd0;
                    state    <= DONE;
                end
            end

            COMP_MUL: begin
                // 乘法单独一拍（×31847 MULT18 + 47bit 比较同拍曾致 100MHz 违例）
                pw_capm_r <= ({15'd0, div_q} * 47'd31847) >> 14;
                pw_indm_r <= ({15'd0, div_q} * 47'd6529)  >> 11;
                state     <= COMP_FIN;
            end

            COMP_FIN: begin
                // C_µF = (Q×16384/V²)×31847>>14；L_0.1mH = (V²/Q)×6529>>11（推导见 AL.md）
                if (pw_lag_r) begin
                    pw_cap_r <= (pw_capm_r > 33'd9999) ? 16'd9999 : pw_capm_r[15:0];
                    pw_ind_r <= 16'd0;
                end else begin
                    pw_ind_r <= (pw_indm_r > 33'd9999) ? 16'd9999 : pw_indm_r[15:0];
                    pw_cap_r <= 16'd0;
                end
                div_task <= 2'd0;
                state    <= DONE;
            end

            DONE: begin
                measure_done_reg <= 1'b1;
                done_toggle_reg  <= ~done_toggle_reg;
                state <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

assign fundamental_freq = fund_freq_reg;
assign rms_va = rms_reg[0];
assign rms_vb = rms_reg[1];
assign rms_vc = rms_reg[2];
assign rms_ia = rms_reg[3];
assign rms_ib = rms_reg[4];
assign rms_ic = rms_reg[5];
assign thd_va = thd_reg[0];
assign thd_vb = thd_reg[1];
assign thd_vc = thd_reg[2];
assign thd_ia = thd_reg[3];
assign thd_ib = thd_reg[4];
assign thd_ic = thd_reg[5];
assign phase_ab = phase_ab_reg;
assign phase_bc = phase_bc_reg;
assign phase_ca = phase_ca_reg;
assign phase_ia = phase_ia_reg;
assign phase_ib = phase_ib_reg;
assign phase_ic = phase_ic_reg;
assign phase_sequence = phase_seq_reg;
assign unbalance_degree = unbalance_reg;
assign measure_done = measure_done_reg;
assign done_toggle = done_toggle_reg;

genvar gi_hm;
generate
    for (gi_hm = 0; gi_hm < HARMONICS_NUM; gi_hm = gi_hm + 1) begin: harm_out
        assign harm_pct_va[gi_hm*16 +: 16] = harm_pct_reg[0][gi_hm];
        assign harm_pct_vb[gi_hm*16 +: 16] = harm_pct_reg[1][gi_hm];
        assign harm_pct_vc[gi_hm*16 +: 16] = harm_pct_reg[2][gi_hm];
        assign harm_pct_ia[gi_hm*16 +: 16] = harm_pct_reg[3][gi_hm];
        assign harm_pct_ib[gi_hm*16 +: 16] = harm_pct_reg[4][gi_hm];
        assign harm_pct_ic[gi_hm*16 +: 16] = harm_pct_reg[5][gi_hm];
    end
endgenerate

endmodule
