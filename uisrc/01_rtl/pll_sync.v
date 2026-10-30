module pll_sync #(
    parameter CLK_FREQ          = 100_000_000,
    parameter SAMPLES_PER_CYCLE = 256,
    parameter FREQ_MIN          = 45,
    parameter FREQ_MAX          = 55,
    parameter ADC_BITS          = 12
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [ADC_BITS-1:0] adc_data,
    input  wire [2:0]         adc_channel,
    input  wire               adc_valid,

    output wire        sample_pulse,
    output wire [31:0] freq_out,
    output wire [15:0] phase_out,
    output wire        locked,

    input  wire        enable
);

// ------------------------------------------------------------
// 全数字锁相环（软件 PLL）
//  - nco_phase：32bit，累计 2^32 = 一个工频周期
//  - 每跨越一个 2^24 相位边界（1/256 周期）产生一个 sample_pulse
//    → 50Hz 时 256 点/周期 = 12.8kHz 采样，与 FFT 窗口严格相干
//  - 输入 A 相（channel 0）过零时刻 err = -nco_phase，
//    PI 调节 nco_increment 使 NCO 相位零点对齐输入过零
//  - 无信号（>2.7 周期无过零）自动回 50Hz 名义速率自由运行
// ------------------------------------------------------------
localparam FREQ_NOMINAL = 50;

localparam [63:0] INC_MIN_W   = (64'd1 << 32) * FREQ_MIN     / CLK_FREQ;
localparam [63:0] INC_NOM_W   = (64'd1 << 32) * FREQ_NOMINAL / CLK_FREQ;
localparam [63:0] INC_MAX_W   = (64'd1 << 32) * FREQ_MAX     / CLK_FREQ;
localparam [31:0] INC_MIN     = INC_MIN_W[31:0];
localparam [31:0] INC_NOMINAL = INC_NOM_W[31:0];
localparam [31:0] INC_MAX     = INC_MAX_W[31:0];

localparam KP_SHIFT = 21;
localparam KI_SHIFT = 24;

localparam signed [31:0] INT_MAX   =  32'sd1024;
localparam signed [31:0] INT_MIN   = -32'sd1024;
localparam signed [31:0] IDEAD_TH  =  32'sd33554432;   // 2^25  积分死区（滤过零量化抖动）
localparam signed [31:0] LOCK_TH   =  32'sd67108864;   // 2^26 ≈ 5.6deg
localparam signed [31:0] UNLOCK_TH =  32'sd134217728;  // 2^28 ≈ 22.5deg
localparam [23:0] SIG_LOST_TH      =  24'd6_000_000;   // ~2.7 周期无过零

localparam [ADC_BITS:0] ZC_TH = 1 << (ADC_BITS - 1);   // 中值 2048
localparam [ADC_BITS:0] ZC_LO = ZC_TH - 13'd64;        // 滞回下限：低于中线 64 码重新武装
                                                       // 滞回必须大于噪声峰峰值+采样斜率
                                                       //（±24 码噪声相邻两样本可差 48 码再叠加
                                                       //   7 码/样本斜率：32 码会在同一过零处
                                                       //   重复武装/二次触发）；适用信号 ≥±80 码

// --- A 相上升过零检测（滞回式 + 双帧武装 + 触发确认 + 不应期）---
// 无滞回时噪声明近中线的多次来回穿越被逐个计入：PI 随机踢动 → NCO 增量抖动
//（频率读数乱跳、采样脉冲抖动、波形畸变）。低于 ZC_LO 武装、升穿中线触发，
// 一周期至多一次；幅度 <64 码的信号不触发（比噪声抖动更可预期）。
// V24 不应期：零点处的模拟瞬态（偏置节点在信号电流换向时被踢）在过零附近
// 打出 跌破武装线→冲过触发线 的毛刺对。实机 FREQ 44.9↔54.9 打满两个钳位
// 边界需要大相位误差的踢动——下降过零处（180°）的毛刺对触发即得半周期
// 相位误差，4ms 不应期罩不住它。现取 12ms：同时覆盖 45~55Hz 全频段上升/
// 下降两个过零点（55Hz 半周期 9.1ms / 45Hz 半周期 11.1ms），又小于最小
// 周期 22.2ms，正常单次过零不受影响。武装条件本身被不应期门控（V23 版
// 强制清零块写在武装块之前，同拍被后写的武装赋值覆盖，形同虚设）。
// V27 双帧武装+触发确认：去刺钳位（±96 码）后的孤立坏帧仍留一帧台阶——
// 坏帧落在上升沿时钳位值=前帧+96，前帧 ≥1952 即冲过触发线，最多提前 7 帧
//（364µs = 1.2% 相位）触发 = 实机 FREQ 48.8↔50.0 交替乱跳的根因（仿真 s9
// 频率直方图 49.2/50.4 双峰同签名）。修复：①武装需连续 2 帧低于武装线
//（孤立坏帧跌破不再武装——真信号波谷有 ~180 连续帧，无影响）；②触发候选
// 需下一帧仍 ≥ 中线才确认（假穿越的下一帧是真值、多在 2048 以下 → 撤销；
// 残余最早触发 ≤1 帧 = 0.13Hz）。真过零恒定延迟 1 帧（52µs），对 NCO 只是
// 常量相移，无频率偏置。
reg signed [ADC_BITS:0] adc_prev, adc_curr;
reg zc_rise, zc_armed;
reg zc_below_d;                       // 上一帧曾低于武装线（双帧武装）
reg zc_pend;                          // 触发候选，待下一帧确认
reg [20:0] zc_refract;
localparam [20:0] ZC_REFRACT_TH = 21'd1_200_000;  // 12ms @ 100MHz

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        adc_prev <= {(ADC_BITS+1){1'b0}};
        adc_curr <= {(ADC_BITS+1){1'b0}};
        zc_rise  <= 1'b0;
        zc_armed <= 1'b0;
        zc_below_d <= 1'b0;
        zc_pend  <= 1'b0;
        zc_refract <= 21'd0;
    end else begin
        zc_rise <= 1'b0;
        if (zc_refract != 21'd0)
            zc_refract <= zc_refract - 1'b1;
        if (adc_valid && adc_channel == 3'd0) begin
            adc_prev <= adc_curr;
            adc_curr <= {1'b0, adc_data};
            if (zc_pend) begin
                if (adc_curr >= $signed(ZC_TH)) begin
                    zc_rise    <= 1'b1;          // 确认：连续 2 帧在中线上方
                    zc_armed   <= 1'b0;
                    zc_pend    <= 1'b0;
                    zc_refract <= ZC_REFRACT_TH;
                end else begin
                    zc_pend <= 1'b0;             // 假穿越撤销（保持武装等真过零）
                end
            end else begin
                if (zc_refract == 21'd0 && zc_below_d &&
                    adc_curr < $signed(ZC_LO)) begin
                    zc_armed <= 1'b1;            // 双帧武装
                end else if (zc_armed && adc_prev < $signed(ZC_TH) &&
                             adc_curr >= $signed(ZC_TH) && adc_curr > adc_prev) begin
                    zc_pend <= 1'b1;             // 候选：待下一帧确认
                end
            end
            zc_below_d <= (adc_curr < $signed(ZC_LO));
        end
    end
end

// --- NCO 与 PI 环路 ---
reg [31:0] nco_phase;
reg [31:0] nco_increment;
reg signed [31:0] pi_int;
reg [23:0] period_cnt;
reg locked_reg;
reg [3:0] lock_cnt, unlock_cnt;
reg [31:0] freq_reg;
reg sample_pulse_reg;

wire [31:0] nco_next = nco_phase + nco_increment;
wire sample_boundary = (nco_next[31:24] != nco_phase[31:24]);
wire sig_lost = (period_cnt >= SIG_LOST_TH);

// PI 环路拆 7 拍（原过零拍内 32bit 取负+加法+两级饱和+×1e9 乘法
// 共 14.7ns，超 10ns 周期）。过零间隔约 200 万拍，流水延迟无影响。
// 拍1 锁存 err；拍2 积分；拍3a 锁相判定；拍3b 增量三输入求和；
// 拍4 增量饱和；拍5 更新增量；拍6/7 频率读数（32x10 常数乘拆两拍）。
// V19：原拍5 的 32x32 乘法单拍完成是 100MHz 域最深组合路径，实机
// 违例下在过零事件处打出坏增量造成采样栅格跳相（见第十一节）。
reg signed [31:0] err_r, kp_term_r, inc_add_r, inc_add2_r, inc_r;
reg pi_s1, pi_s2, pi_s3, pi_s4;
reg        pi_s5, pi_s6;            // 频率读数两拍流水（V19）
reg [31:0] fm_a;
reg [44:0] fm_p;

wire signed [31:0] ki_term_w = err_r >>> KI_SHIFT;
wire signed [31:0] int_add_w = pi_int + ki_term_w;
wire signed [31:0] int_sat_w = (int_add_w > INT_MAX)  ? INT_MAX  :
                               (int_add_w < INT_MIN)  ? INT_MIN  : int_add_w;
wire signed [31:0] int_nxt_w = (err_r > IDEAD_TH || err_r < -IDEAD_TH)
                               ? int_sat_w : pi_int;
wire signed [31:0] inc_sum_w = $signed(INC_NOMINAL) + kp_term_r + pi_int;
wire signed [31:0] inc_sat_w = (inc_add2_r > $signed(INC_MAX)) ? $signed(INC_MAX) :
                               (inc_add2_r < $signed(INC_MIN)) ? $signed(INC_MIN) : inc_add2_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        nco_phase        <= 32'd0;
        nco_increment    <= INC_NOMINAL;
        pi_int           <= 32'sd0;
        period_cnt       <= 24'd0;
        locked_reg       <= 1'b0;
        lock_cnt         <= 4'd0;
        unlock_cnt       <= 4'd0;
        freq_reg         <= 32'd500;
        sample_pulse_reg <= 1'b0;
        err_r            <= 32'sd0;
        kp_term_r        <= 32'sd0;
        inc_add_r        <= 32'sd0;
        inc_add2_r       <= 32'sd0;
        inc_r            <= 32'sd0;
        pi_s1            <= 1'b0;
        pi_s2            <= 1'b0;
        pi_s3            <= 1'b0;
        pi_s4            <= 1'b0;
        pi_s5            <= 1'b0;
        pi_s6            <= 1'b0;
        fm_a             <= 32'd0;
        fm_p             <= 45'd0;
    end else begin
        sample_pulse_reg <= 1'b0;
        nco_phase        <= nco_next;
        if (sample_boundary)
            sample_pulse_reg <= 1'b1;

        if (period_cnt < 24'hFFFFFF)
            period_cnt <= period_cnt + 1'b1;

        pi_s1 <= 1'b0;
        pi_s2 <= 1'b0;
        pi_s3 <= 1'b0;
        pi_s4 <= 1'b0;

        if (sig_lost) begin
            // 信号丢失：回名义速率自由运行（示波器仍可看波形）
            locked_reg    <= 1'b0;
            lock_cnt      <= 4'd0;
            unlock_cnt    <= 4'd0;
            pi_int        <= 32'sd0;
            nco_increment <= INC_NOMINAL;
            freq_reg      <= 32'd500;
            period_cnt    <= 24'd0;
        end else begin
            // 拍1：目标输入过零时 nco_phase = 0（mod 2^32）
            if (zc_rise && enable) begin
                period_cnt <= 24'd0;
                err_r      <= -$signed(nco_phase);
                pi_s1      <= 1'b1;
            end

            // 拍2：积分项（死区 + 饱和）
            if (pi_s1) begin
                pi_int    <= int_nxt_w;
                kp_term_r <= err_r >>> KP_SHIFT;
                pi_s2     <= 1'b1;
            end

            // 拍3a：锁相判定；拍3b：三输入求和寄存；拍4：增量饱和
            if (pi_s2) begin
                if (err_r > -LOCK_TH && err_r < LOCK_TH) begin
                    unlock_cnt <= 4'd0;
                    if (lock_cnt < 4'd6)
                        lock_cnt <= lock_cnt + 1'b1;
                    else
                        locked_reg <= 1'b1;
                end else begin
                    lock_cnt <= 4'd0;
                    if (err_r > UNLOCK_TH || err_r < -UNLOCK_TH) begin
                        if (unlock_cnt < 4'd4)
                            unlock_cnt <= unlock_cnt + 1'b1;
                        else
                            locked_reg <= 1'b0;
                    end else begin
                        unlock_cnt <= 4'd0;
                    end
                end
            end

            if (pi_s2) begin
                inc_add_r  <= inc_sum_w;      // 三输入求和单独一拍
                pi_s3      <= 1'b1;
            end else if (pi_s3) begin
                inc_add2_r <= inc_add_r;      // 再寄一拍给饱和比较满周期裕量
                pi_s3      <= 1'b0;
                pi_s4      <= 1'b1;
            end

            // 拍4：增量饱和；同时装载频率乘法输入寄存器
            // V19：原拍5单拍内做 32x32→64 乘法（1e9*inc_r>>32）是 100MHz 域
            // 最深组合路径——实机时序违例下，每次过零事件（全芯片开关活动
            // 峰值）此处打出坏增量 → NCO 相位单拍跳 +30~45° → 采样栅格每
            // 周期跳相（实机外接正弦"固定相位垂直跳变、THD 114%"实photo）。
            // 现 32x10 常数乘(954/4096=0.23291≈1e9/2^32)与移位拆两拍。
            // 注意 nco_increment/fm_a 必须取当拍 inc_sat_w（非阻塞下读旧
            // inc_r 会滞后一个过零周期，环路跟踪失效）
            if (pi_s4) begin
                inc_r         <= inc_sat_w;
                nco_increment <= inc_sat_w;
                fm_a          <= inc_sat_w;
                pi_s5         <= 1'b1;
            end else if (pi_s5) begin
                fm_p  <= fm_a * 13'd954;
                pi_s5 <= 1'b0;
                pi_s6 <= 1'b1;
            end else if (pi_s6) begin
                freq_reg <= fm_p[43:12];      // >>12 取上段（0.1Hz 单位）
                pi_s6    <= 1'b0;
            end
        end
    end
end

assign sample_pulse = sample_pulse_reg;
assign freq_out     = freq_reg;
assign phase_out    = nco_phase[31:16];
assign locked       = locked_reg;

endmodule
