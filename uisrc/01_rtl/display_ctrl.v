// HDMI 显示控制：720P@60 时序 + 三相电能质量分析仪界面（仪表级深色主题）
//
// 布局（1280×720）：
//   顶栏 y0..31   ：16×32 标题（青）、模式胶囊、功能标签、RUN/STOP、ALARM
//   左面板 x0..207：大字数据带（Ua/Ia/THDa、Ub/Ib/THDb、Uc/Ic/THDc、
//                   FREQ、UNB、PF、PTOT、QTOT、COMP），左缘相色条；
//                   底部小字区：SEQ 相序、PH 相位差、VI 功率因数角
//   主区 x216..1279（VIEW 0→2 循环，功率数据固定在左面板不再单独占页）：
//     0 示波：A/B/C 三窗上下，V 相色 3px 霓虹 + I 琥珀 1px，
//             左缘电压刻度（6 字符）、右缘电流刻度，A/V/I 字符牌右移避让刻度
//     1 频谱：15 组×3 相柱状图（顶部亮帽）
//     2 谐波表：16×32 大字（表框/表头/行线适配）
//
// 双字库：font_rom 8×16（小字）、font2_rom 16×32（大字）
// 文本 RAM：top_ram 顶栏、panel_ram 面板小字、lbl_ram 大字标签、
//           val_ram 大字数值、table_ram 谐波表（16×64 大字）
// 流水线：c1 文本 RAM/波形 RAM 地址 → c2 字符选择+双字库读+波形缩放 → c3 取位+几何+RGB
module display_ctrl #(
    parameter H_RES = 1280,
    parameter V_RES = 720,
    parameter DATA_BITS = 12,
    parameter HARMONICS_NUM = 15
)(
    input  wire        clk,               // 74.25MHz 像素时钟
    input  wire        rst_n,
    output reg         vid_hs,
    output reg         vid_vs,
    output reg         vid_de,
    output reg  [7:0]  rgb_r,
    output reg  [7:0]  rgb_g,
    output reg  [7:0]  rgb_b,
    input  wire [31:0] rms_va, rms_vb, rms_vc,
    input  wire [31:0] rms_ia, rms_ib, rms_ic,
    input  wire [31:0] thd_va, thd_vb, thd_vc,
    input  wire [31:0] thd_ia, thd_ib, thd_ic,
    input  wire [31:0] fundamental_freq,
    input  wire [HARMONICS_NUM*16-1:0] harm_pct_va, harm_pct_vb, harm_pct_vc,
    input  wire [HARMONICS_NUM*16-1:0] harm_pct_ia, harm_pct_ib, harm_pct_ic,
    input  wire [15:0] phase_ab, phase_bc, phase_ca,
    input  wire [15:0] phase_ia, phase_ib, phase_ic,
    input  wire        phase_sequence,
    input  wire [31:0] unbalance_degree,
    input  wire        done_toggle,
    input  wire        wf_toggle,
    input  wire [DATA_BITS-1:0] wf_va, wf_vb, wf_vc,
    input  wire [DATA_BITS-1:0] wf_ia, wf_ib, wf_ic,
    input  wire        mode_toggle,
    input  wire        freeze,
    input  wire [2:0]  event_alarm,
    input  wire        func_toggle,
    input  wire [10:0] func_data,         // {func[1:0], chan[1:0], vi[1:0], self[2:0], pick[1:0]}
    input  wire        view_toggle,
    input  wire [44:0] view_data,         // {tb s3, pan9, vpos×3 s8, soff×3 s3}
    input  wire [177:0] pwr_bus,          // {pfa15, pa,pb,pc16, qa,qb,qc16, ptot,qtot16, cap,ind16, 3flags}
    input  wire        enable
);

// ---------------------------------------------------------------------------
// 配色
// ---------------------------------------------------------------------------
localparam [23:0] BG_MAIN   = 24'h0B0E16;
localparam [23:0] BG_TOP    = 24'h0E1526;
localparam [23:0] BG_PANEL  = 24'h11151F;
localparam [23:0] BG_TABLE  = 24'h0F1322;
localparam [23:0] LINE_SEP  = 24'h2A3348;
localparam [23:0] LINE_DIV  = 24'h1E2434;
localparam [23:0] GRID_V    = 24'h141926;
localparam [23:0] GRID_VM   = 24'h1C2336;
localparam [23:0] GRID_H    = 24'h181F30;
localparam [23:0] GRID_MID  = 24'h222B42;
localparam [23:0] AXIS      = 24'h3A445C;
localparam [23:0] ACCENT    = 24'h35D0FF;
localparam [23:0] ACC_LN    = 24'h2BA8D8;
localparam [23:0] ACC_LN2   = 24'h14425A;
localparam [23:0] TXT_BRIGHT= 24'hF2F4F8;
localparam [23:0] TXT_BODY  = 24'h9AA3B5;
localparam [23:0] TXT_DIM   = 24'h6A7386;
localparam [23:0] TXT_FAINT = 24'h4A5264;
localparam [23:0] COL_A  = 24'hFF5555, GLOW_A = 24'h6D2B2B, CAP_A = 24'hFFA2A2;
localparam [23:0] ACC_A  = 24'h9D393C, TXT_A  = 24'hFF8585, LBL_A = 24'hB06060;
localparam [23:0] COL_B  = 24'h40E070, GLOW_B = 24'h20623A, CAP_B = 24'h96EEB1;
localparam [23:0] ACC_B  = 24'h2B8C4C, TXT_B  = 24'h71E896, LBL_B = 24'h60B078;
localparam [23:0] COL_C  = 24'h5076FF, GLOW_C = 24'h273873, CAP_C = 24'h96B4FF;
localparam [23:0] ACC_C  = 24'h344CA2, TXT_C  = 24'h86A4FF, LBL_C = 24'h6078C0;
localparam [23:0] COL_I  = 24'hFFC84C, GLOW_I = 24'h5C4517;   // 电流琥珀
localparam [23:0] ACC_N  = 24'h1E4E5E;
localparam [23:0] CHIP_DIM   = 24'h2E323E;
localparam [23:0] CHIP_V     = 24'h9AA3B5, CHIP_V_TXT = 24'h1A1E26;
localparam [23:0] CHIP_I     = 24'hFFC84C, CHIP_I_TXT = 24'h2A2008;
localparam [23:0] CHIP_V_DIM = 24'h3A4050;
localparam [23:0] PILL0_FILL = 24'h35D0FF, PILL0_BORD = 24'h8FE8FF;
localparam [23:0] PILL1_FILL = 24'hFFB454, PILL1_BORD = 24'hFFD9A0;
localparam [23:0] PILL2_FILL = 24'hC792EA, PILL2_BORD = 24'hE2C2F5;
localparam [23:0] RUN_FILL  = 24'h58FF58, RUN_BORD  = 24'hAEFFAE, RUN_TXT  = 24'h06280F;
localparam [23:0] STOP_FILL = 24'hFF5858, STOP_BORD = 24'hFFAEAE, STOP_TXT = 24'h2A0808;
localparam [23:0] ALM_FILL  = 24'hE82828, ALM_BORD  = 24'hFF8888, ALM_TXT  = 24'hFFF2F2;
localparam [23:0] ALM_DIMF  = 24'h4A1010, ALM_DIMB  = 24'h7A2830, ALM_DIMT = 24'hC89098;
localparam [23:0] ALM_OFFF  = 24'h161A26, ALM_OFFB  = 24'h3A2430, ALM_OFFT = 24'h7A4048;
localparam [23:0] TBL_HDRBG = 24'h1A2338, TBL_HDRTX = 24'hF2D488;
localparam [23:0] TBL_ABG   = 24'h1A1420, TBL_BBG = 24'h13201A, TBL_CBG = 24'h151A2C;
localparam [23:0] TBL_LINE  = 24'h1D2436;

// ---------------------------------------------------------------------------
// 1. 720P@60 时序
// ---------------------------------------------------------------------------
localparam H_FRONT_PORCH = 110, H_SYNC_WIDTH = 40, H_BACK_PORCH = 220;
localparam H_TOTAL = H_RES + H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH;
localparam V_FRONT_PORCH = 5, V_SYNC_WIDTH = 5, V_BACK_PORCH = 20;
localparam V_TOTAL = V_RES + V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH;

reg [11:0] h_counter;
reg [10:0] v_counter;
reg hs_int, vs_int, de_int;
wire [11:0] h_pos = h_counter - (H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH);
wire [10:0] v_pos = v_counter - (V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH);
wire in_active = (h_counter >= (H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH)) &&
                 (h_counter < (H_FRONT_PORCH + H_SYNC_WIDTH + H_BACK_PORCH + H_RES)) &&
                 (v_counter >= (V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH)) &&
                 (v_counter < (V_FRONT_PORCH + V_SYNC_WIDTH + V_BACK_PORCH + V_RES));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        h_counter <= 12'd0; v_counter <= 11'd0;
        hs_int <= 1'b0; vs_int <= 1'b0; de_int <= 1'b0;
    end else begin
        if (h_counter >= H_TOTAL - 1) begin
            h_counter <= 12'd0;
            if (v_counter >= V_TOTAL - 1) v_counter <= 11'd0;
            else v_counter <= v_counter + 1'b1;
        end else h_counter <= h_counter + 1'b1;
        hs_int <= (h_counter >= H_FRONT_PORCH) && (h_counter < H_FRONT_PORCH + H_SYNC_WIDTH);
        vs_int <= (v_counter >= V_FRONT_PORCH) && (v_counter < V_FRONT_PORCH + V_SYNC_WIDTH);
        de_int <= in_active;
    end
end

reg [11:0] hp_d1, hp_d2, hp_d3;
reg [10:0] vp_d1, vp_d2, vp_d3;
reg hs_d1, hs_d2, vs_d1, vs_d2, de_d1, de_d2;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hp_d1 <= 12'd0; hp_d2 <= 12'd0; hp_d3 <= 12'd0;
        vp_d1 <= 11'd0; vp_d2 <= 11'd0; vp_d3 <= 11'd0;
        hs_d1 <= 1'b0; hs_d2 <= 1'b0; vid_hs <= 1'b0;
        vs_d1 <= 1'b0; vs_d2 <= 1'b0; vid_vs <= 1'b0;
        de_d1 <= 1'b0; de_d2 <= 1'b0; vid_de <= 1'b0;
    end else begin
        hp_d1 <= h_pos;  vp_d1 <= v_pos;
        hp_d2 <= hp_d1;  vp_d2 <= vp_d1;
        hp_d3 <= hp_d2;  vp_d3 <= vp_d2;
        hs_d1 <= hs_int;  hs_d2 <= hs_d1;  vid_hs <= hs_d2;
        vs_d1 <= vs_int;  vs_d2 <= vs_d1;  vid_vs <= vs_d2;
        de_d1 <= de_int;  de_d2 <= de_d1;  vid_de <= de_d2;
    end
end
wire act_d3 = de_d2;

// ---------------------------------------------------------------------------
// 2. CDC：测量结果 / 波形 / 模式 / 功能 / 视图 / 功率
// ---------------------------------------------------------------------------
reg done_tg_m, done_tg_s, done_tg_p;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin done_tg_m <= 0; done_tg_s <= 0; done_tg_p <= 0; end
    else begin done_tg_m <= done_toggle; done_tg_s <= done_tg_m; done_tg_p <= done_tg_s; end
end
wire meas_update = done_tg_s ^ done_tg_p;

reg wf_tg_m, wf_tg_s, wf_tg_p;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin wf_tg_m <= 0; wf_tg_s <= 0; wf_tg_p <= 0; end
    else begin wf_tg_m <= wf_toggle; wf_tg_s <= wf_tg_m; wf_tg_p <= wf_tg_s; end
end

reg frz_m, frz_s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin frz_m <= 0; frz_s <= 0; end
    else begin frz_m <= freeze; frz_s <= frz_m; end
end
wire wf_sample = (wf_tg_s ^ wf_tg_p) && !frz_s;
wire meas_upd_ok = meas_update && !frz_s;

reg mp_m, mp_s, mp_p;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin mp_m <= 0; mp_s <= 0; mp_p <= 0; end
    else begin mp_m <= mode_toggle; mp_s <= mp_m; mp_p <= mp_s; end
end
wire mode_change = mp_s ^ mp_p;
reg [1:0] mode_px;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) mode_px <= 2'd0;
    else if (mode_change) mode_px <= (mode_px == 2'd2) ? 2'd0 : mode_px + 2'd1;
end

reg ftg_m, ftg_s, ftg_p;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin ftg_m <= 0; ftg_s <= 0; ftg_p <= 0; end
    else begin ftg_m <= func_toggle; ftg_s <= ftg_m; ftg_p <= ftg_s; end
end
wire func_upd = ftg_s ^ ftg_p;
reg [10:0] func_px;
always @(posedge clk) if (func_upd) func_px <= func_data;

reg vtg_m, vtg_s, vtg_p;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin vtg_m <= 0; vtg_s <= 0; vtg_p <= 0; end
    else begin vtg_m <= view_toggle; vtg_s <= vtg_m; vtg_p <= vtg_s; end
end
wire view_upd = vtg_s ^ vtg_p;
reg [44:0] view_px;
reg [10:0] mid_px [0:2];
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        view_px <= 45'd0;
        mid_px[0] <= 11'd154; mid_px[1] <= 11'd382; mid_px[2] <= 11'd610;
    end else if (view_upd) begin
        view_px  <= view_data;
        mid_px[0] <= 11'd154 + {{3{view_data[32]}}, view_data[32:25]};
        mid_px[1] <= 11'd382 + {{3{view_data[24]}}, view_data[24:17]};
        mid_px[2] <= 11'd610 + {{3{view_data[16]}}, view_data[16:9]};
    end
end
wire signed [2:0] tb_px = view_px[44:42];
wire [8:0] pan_px = view_px[41:33];
wire signed [2:0] soff_px0 = view_px[8:6], soff_px1 = view_px[5:3], soff_px2 = view_px[2:0];

reg [177:0] pwr_px;
always @(posedge clk) pwr_px <= pwr_bus;
wire [15:0] pw_pa_w = pwr_px[162:147];
wire [15:0] pw_qa_w = pwr_px[114:99];
wire [15:0] pw_ptot_w = pwr_px[66:51];
wire [15:0] pw_qtot_w = pwr_px[50:35];
wire [15:0] pw_cap_w = pwr_px[34:19];
wire [15:0] pw_ind_w = pwr_px[18:3];
wire pw_cpres = pwr_px[2];
wire pw_pfok  = pwr_px[1];
wire pw_lag   = pwr_px[0];
wire [14:0] pw_pf_a = pwr_px[177:163];
// PF×100 二进制（0..99）→ 十位/个位 BCD（旧代码直接把二进制按 BCD 拆半字节，99 显示成 "63"）。
// 乘法必须经寄存器分级（单拍级联常数乘法链曾致 75MHz 域 -8.6ns）：
// pf_bin 已是寄存器，pf_m 一拍乘法出十位；下一拍从寄存后的十位算个位
wire [19:0] pf_mul = pw_pf_a[13:0] * 7'd25;
reg [6:0] pf_bin;
always @(posedge clk) pf_bin <= (pw_pf_a > 15'd16383) ? 7'd99 : pf_mul[19:12];
wire [14:0] pf_m    = pf_bin * 8'd205;
reg [3:0]  pf_tens_r;
wire [6:0] pf_t10_w = pf_tens_r * 7'd10;
reg [3:0]  pf_ones_r;
always @(posedge clk) begin
    pf_tens_r <= pf_m[14:11];
    pf_ones_r <= pf_bin - pf_t10_w;
end

wire [1:0] fmode_px = func_px[10:9];
wire [1:0] chan_eff = (fmode_px == 2'd1) ? 2'd1 : func_px[8:7];
wire [1:0] vi_px    = func_px[6:5];
wire show_v = (vi_px != 2'd2);
wire show_i = (vi_px != 2'd1);
wire vi_cur = (vi_px == 2'd2);
wire [1:0] pick_px = func_px[1:0];

reg alm_m, alm_s0; reg [2:0] alm_m2, alm_s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin alm_m <= 0; alm_s0 <= 0; alm_m2 <= 3'd0; alm_s <= 3'd0; end
    else begin alm_m <= event_alarm[0]; alm_s0 <= alm_m; alm_m2 <= event_alarm; alm_s <= alm_m2; end
end
wire alm_act = (alm_s != 3'd0);

// 报警闪烁
reg [24:0] blink_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) blink_cnt <= 25'd0;
    else blink_cnt <= blink_cnt + 1'b1;
end
wire blink_on = blink_cnt[24];

// 像素域锁存测量结果
reg [15:0] rms_px [0:2], thd_px [0:2];
reg [15:0] rms_i_px [0:2], thd_i_px [0:2];
reg [15:0] freq_px, unb_px;
// 谐波值存双 BRAM 备份（频谱柱读、谐波表 BCD 读各占独立读口）。
// 旧实现 = 96×16 寄存器阵列 + 90 项同拍并行写 + 两处 96:1 组合读复用器，
// 是 measurement/display 交界布线拥塞（PHY-8023 聚簇于 harm_px/fmt_val_b）的主源；
// BRAM 独占走线列并吞掉整个复用器森林。写侧串行化：90 项每拍写 1 项，
// 输入总线在两轮测量间稳定，1.2µs 写完远快于测量节拍。
// BRAM 深度必须取 2 的幂：TD 5.6 推断器遇到非 2 幂深度（如 96）的可写 RAM
// 会在 elaborate 阶段 Signal Abort（2026-09-20 实测），故取 128、高端 32 项空置
(* ram_style = "block" *) reg [15:0] harm_sp_ram [0:127];  // 频谱柱读
(* ram_style = "block" *) reg [15:0] harm_fm_ram [0:127];  // 谐波表 BCD 读
reg [15:0] pdeg_px [0:2], pdeg_i_px [0:2];
reg pneg_px [0:2], pneg_i_px [0:2];
reg seq_px;

wire [31:0] rms_cv_a = (rms_va > 32'd9999) ? 32'd9999 : rms_va;
wire [31:0] rms_cv_b = (rms_vb > 32'd9999) ? 32'd9999 : rms_vb;
wire [31:0] rms_cv_c = (rms_vc > 32'd9999) ? 32'd9999 : rms_vc;
function signed [16:0] fold16;
    input [15:0] p;
    fold16 = (p >= 16'h8000) ? ($signed({1'b0, p}) - 17'sd65536) : $signed({1'b0, p});
endfunction
wire signed [16:0] pab_f = fold16(phase_ab), pbc_f = fold16(phase_bc), pca_f = fold16(phase_ca);
wire signed [16:0] pab_a = (pab_f < 0) ? -pab_f : pab_f;
wire signed [16:0] pbc_a = (pbc_f < 0) ? -pbc_f : pbc_f;
wire signed [16:0] pca_a = (pca_f < 0) ? -pca_f : pca_f;
wire [8:0] pab_d = {1'b0, pab_a * 23'sd45 >> 13};
wire [8:0] pbc_d = {1'b0, pbc_a * 23'sd45 >> 13};
wire [8:0] pca_d = {1'b0, pca_a * 23'sd45 >> 13};
wire signed [16:0] pia_f = fold16(phase_ia), pib_f = fold16(phase_ib), pic_f = fold16(phase_ic);
wire signed [16:0] pia_a = (pia_f < 0) ? -pia_f : pia_f;
wire signed [16:0] pib_a = (pib_f < 0) ? -pib_f : pib_f;
wire signed [16:0] pic_a = (pic_f < 0) ? -pic_f : pic_f;
wire [8:0] pia_d = {1'b0, pia_a * 23'sd45 >> 13};
wire [8:0] pib_d = {1'b0, pib_a * 23'sd45 >> 13};
wire [8:0] pic_d = {1'b0, pic_a * 23'sd45 >> 13};

// V37：开路/无效电压判据（用户指令：低于 10V 的都是检测错误的波形，不纳入
// 计算与显示）——RMS-V<10.0V 视为无信号：读数 0.0V、THD 0.0、迹线平线、
// 频谱/谐波表全 0（写侧门控），各页面一致不再闪烁。真实信号须 ≥10V RMS
//（≈±14V 峰，推荐 ±20V 演示）；若需测更小信号把此常数调小
localparam [31:0] V_OPEN_RMS_TH = 32'd100;   // 10.0V（0.1V 单位 100）
always @(posedge clk) begin
    if (meas_upd_ok) begin
        rms_px[0] <= (rms_va < V_OPEN_RMS_TH) ? 16'd0 : rms_cv_a[15:0];
        rms_px[1] <= (rms_vb < V_OPEN_RMS_TH) ? 16'd0 : rms_cv_b[15:0];
        rms_px[2] <= (rms_vc < V_OPEN_RMS_TH) ? 16'd0 : rms_cv_c[15:0];
        // V35：I 路读数开路门控——RMS-I < 0.5A 显示 0.0（仪器语义本就以 0.5A
        // 判定电流有无，PF/COMP 的 pw_cpres 同阈值）；顺带消灭 0.3A↔0.4A 跨
        // bucket_i 挡边界造成的 0.5A↔2A 挡位跳
        rms_i_px[0] <= (rms_ia < 32'd5) ? 16'd0 : rms_ia[15:0];
        rms_i_px[1] <= (rms_ib < 32'd5) ? 16'd0 : rms_ib[15:0];
        rms_i_px[2] <= (rms_ic < 32'd5) ? 16'd0 : rms_ic[15:0];
        // THD 开路门控（V35 引入、V37 阈值随读数同步 10V）：THD=谐波和/基波，
        // 无信号通道基波≈0 → 噪声/噪声相除 → 乱跳。基波都没了 THD 无定义，
        // 仪器语义应为 0（与 PF 无电流显 "--" 同哲学）
        thd_px[0] <= (rms_va < V_OPEN_RMS_TH) ? 16'd0 : (thd_va > 32'd9999) ? 16'd9999 : thd_va[15:0];
        thd_px[1] <= (rms_vb < V_OPEN_RMS_TH) ? 16'd0 : (thd_vb > 32'd9999) ? 16'd9999 : thd_vb[15:0];
        thd_px[2] <= (rms_vc < V_OPEN_RMS_TH) ? 16'd0 : (thd_vc > 32'd9999) ? 16'd9999 : thd_vc[15:0];
        thd_i_px[0] <= (rms_ia < 32'd5) ? 16'd0 : (thd_ia > 32'd9999) ? 16'd9999 : thd_ia[15:0];
        thd_i_px[1] <= (rms_ib < 32'd5) ? 16'd0 : (thd_ib > 32'd9999) ? 16'd9999 : thd_ib[15:0];
        thd_i_px[2] <= (rms_ic < 32'd5) ? 16'd0 : (thd_ic > 32'd9999) ? 16'd9999 : thd_ic[15:0];
        freq_px <= (fundamental_freq > 32'd65535) ? 16'd65535 : fundamental_freq[15:0];
        unb_px <= (unbalance_degree > 32'd9999) ? 16'd9999 : unbalance_degree[15:0];
        pdeg_px[0] <= {7'd0, pab_d}; pdeg_px[1] <= {7'd0, pbc_d}; pdeg_px[2] <= {7'd0, pca_d};
        pdeg_i_px[0] <= {7'd0, pia_d}; pdeg_i_px[1] <= {7'd0, pib_d}; pdeg_i_px[2] <= {7'd0, pic_d};
        pneg_px[0] <= pab_f[16]; pneg_px[1] <= pbc_f[16]; pneg_px[2] <= pca_f[16];
        pneg_i_px[0] <= pia_f[16]; pneg_i_px[1] <= pib_f[16]; pneg_i_px[2] <= pic_f[16];
        seq_px <= phase_sequence;
    end
end

// ---------------------------------------------------------------------------
// 3. 波形写指针 + 时基抽点 + 挡位判定（自适应刻度）
// ---------------------------------------------------------------------------
// 时基抽点：环形缓冲恒 512 深，写侧按窗长分频存点——tb=0 每 sample 存 1 点
//（512 点=2 周期），tb=+1 每 2 点取 1（=4 周期），tb=+2 每 4 点取 1（=8 周期）。
// 256/分频仍为整周期 → 环形回读天然无缝；读侧 512 地址恒等于整窗。
// （旧实现写侧恒速、读侧把地址左移 1/2 位：4/8 周期档只读出缓冲前 1/2/1/4，
//   然后地址回绕把同一段 2 周期旧数据重复 2/4 遍——即"显示一半截断又从头
//   来一遍"的实机故障）
// wf_dec = D-1（装载后倒数 D-1 个样本后回到 0 再捕获）：÷2 装载 1、÷4 装载 3，
// 否则 ÷4 实得 ÷3（2→1→0 只跳过 2 个样本）
wire [1:0] wf_dec = (tb_px == 3'sd1) ? 2'd1 : (tb_px == 3'sd2) ? 2'd3 : 2'd0;
reg  [1:0] wf_div;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)        wf_div <= 2'd0;
    else if (view_upd) wf_div <= 2'd0;    // 时基切换：下一样本重新起组
    else if (wf_sample)
        wf_div <= (wf_div == 2'd0) ? wf_dec : wf_div - 2'd1;
end
wire wf_capture = wf_sample && (wf_div == 2'd0);

reg [8:0] wf_wr;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) wf_wr <= 9'd0;
    else if (wf_capture) wf_wr <= wf_wr + 1'b1;
end

// 挡位由每相 RMS（0.1V/0.1A，像素域已有）直接判定，估算峰峰 = 2.28×RMS码
//（2√2×1.15 裕量，谐波波峰容忍 15%）。电压挡 p-p:5/10/15/25/35/50/70/100V，
// 电流挡 p-p:0.5/1/2/5/10/15/25/40A；旋转 ENC4 时增益在挡位上再放大。
function [2:0] bucket_v;
    input [15:0] rms;
    bucket_v = (rms >= 16'd216) ? 3'd7 :   // ≥21.6V RMS → 100V 挡
               (rms >= 16'd154) ? 3'd6 :   // ≥15.4V → 70V 挡
               (rms >= 16'd108) ? 3'd5 :   // ≥10.8V → 50V 挡
               (rms >= 16'd77)  ? 3'd4 :   // ≥7.7V  → 35V 挡
               (rms >= 16'd46)  ? 3'd3 :   // ≥4.6V  → 25V 挡
               (rms >= 16'd31)  ? 3'd2 :   // ≥3.1V  → 15V 挡
               (rms >= 16'd16)  ? 3'd1 :   // ≥1.6V  → 10V 挡
                                3'd0;      //          → 5V 挡
endfunction
function [2:0] bucket_i;
    input [15:0] rms;
    bucket_i = (rms >= 16'd77) ? 3'd7 :   // ≥7.7A RMS → 40A 挡
               (rms >= 16'd47) ? 3'd6 :   // ≥4.7A → 25A 挡
               (rms >= 16'd31) ? 3'd5 :   // ≥3.1A → 15A 挡
               (rms >= 16'd16) ? 3'd4 :   // ≥1.6A → 10A 挡
               (rms >= 16'd7)  ? 3'd3 :   // ≥0.7A → 5A 挡
               (rms >= 16'd4)  ? 3'd2 :   // ≥0.4A → 2A 挡
               (rms >= 16'd2)  ? 3'd1 :   // ≥0.2A → 1A 挡
                                3'd0;      //         → 0.5A 挡
endfunction
reg [2:0] scl_v_r [0:2];
reg [2:0] scl_i_r [0:2];
integer wi;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (wi = 0; wi < 3; wi = wi + 1) begin
            scl_v_r[wi] <= 3'd7;            // 复位=最大挡
            scl_i_r[wi] <= 3'd7;
        end
    end else if (meas_upd_ok) begin
        scl_v_r[0] <= bucket_v(rms_px[0]);
        scl_v_r[1] <= bucket_v(rms_px[1]);
        scl_v_r[2] <= bucket_v(rms_px[2]);
        scl_i_r[0] <= bucket_i(rms_i_px[0]);
        scl_i_r[1] <= bucket_i(rms_i_px[1]);
        scl_i_r[2] <= bucket_i(rms_i_px[2]);
    end
end

// ENC4 手动量程：有效挡位 = 自适应挡位 + 偏移（饱和到 0..7）。
// 旧实现 ENC4 是挡位上的固定 1x/2x/4x 增益——自适应挡位恒把波形放到 ~62% 半窗，
// 2x 起即削顶、4x 与 2x 视觉无差异（"拧了没反应"）；改为 V/DIV 式挡位偏移后
// 每拧一格波形按挡位比 (~1.4x) 缩放，任何幅度都可见可控
reg signed [2:0] s0_d, s1_d, s2_d;         // 偏移寄存（c2b 拍与 mid_d 同步）
function [2:0] scl_eff;
    input [2:0] base;
    input signed [2:0] off;
    reg signed [4:0] s;
    begin
        // off 必须符号扩展（{{2{off[2]}}, off}）：{2'b00, off} 会把 -4 变 +4
        s = $signed({1'b0, base}) + $signed({{2{off[2]}}, off});
        scl_eff = (s < 5'sd0) ? 3'd0 : (s > 5'sd7) ? 3'd7 : s[2:0];
    end
endfunction
wire [2:0] ev0  = scl_eff(scl_v_r[0], s0_d), ev1  = scl_eff(scl_v_r[1], s1_d), ev2  = scl_eff(scl_v_r[2], s2_d);
wire [2:0] eiv0 = scl_eff(scl_i_r[0], s0_d), eiv1 = scl_eff(scl_i_r[1], s1_d), eiv2 = scl_eff(scl_i_r[2], s2_d);

// ---------------------------------------------------------------------------
// 4. 文本 RAM + 双字库
// ---------------------------------------------------------------------------
(* ram_style = "block" *) reg [7:0]  font_rom  [0:2047];
(* ram_style = "block" *) reg [15:0] font2_rom [0:4095];
(* ram_style = "block" *) reg [7:0] top_ram   [0:511];
(* ram_style = "block" *) reg [7:0] panel_ram [0:1439];
(* ram_style = "block" *) reg [7:0] lbl_ram   [0:127];
(* ram_style = "block" *) reg [7:0] val_ram   [0:127];
(* ram_style = "block" *) reg [7:0] table_ram [0:1023];

initial begin `include "font_init.vh" end
initial begin `include "font2_init.vh" end

integer ii, jj;
initial begin : text_init
    for (ii = 0; ii < 512; ii = ii + 1) top_ram[ii] = 8'h20;
    for (ii = 0; ii < 1440; ii = ii + 1) panel_ram[ii] = 8'h20;
    for (ii = 0; ii < 128; ii = ii + 1) lbl_ram[ii] = 8'h20;
    for (ii = 0; ii < 128; ii = ii + 1) val_ram[ii] = 8'h20;
    for (ii = 0; ii < 1024; ii = ii + 1) table_ram[ii] = 8'h20;
    for (ii = 0; ii < 3; ii = ii + 1) begin
        rms_px[ii] = 16'd0; thd_px[ii] = 16'd0;
        rms_i_px[ii] = 16'd0; thd_i_px[ii] = 16'd0;
        pdeg_px[ii] = 16'd0; pdeg_i_px[ii] = 16'd0;
        pneg_px[ii] = 1'b0; pneg_i_px[ii] = 1'b0;
    end
    freq_px = 16'd0; unb_px = 16'd0; seq_px = 1'b0;
    func_px = 11'd0; view_px = 45'd0;
    mid_px[0] = 11'd154; mid_px[1] = 11'd382; mid_px[2] = 11'd610;
    // 谐波 BRAM 不做 initial 清零（TD 对可写 RAM 的 initial 推断不稳）：
    // 上电首帧频谱/谐波表可能显随机值，首次测量更新（<100ms）后自愈

    // 顶栏：标题 16×32（列 0..29）
    top_ram[0*256+0]="3"; top_ram[0*256+1]="-"; top_ram[0*256+2]="P"; top_ram[0*256+3]="H";
    top_ram[0*256+4]="A"; top_ram[0*256+5]="S"; top_ram[0*256+6]="E"; top_ram[0*256+7]=" ";
    top_ram[0*256+8]="P"; top_ram[0*256+9]="O"; top_ram[0*256+10]="W"; top_ram[0*256+11]="E";
    top_ram[0*256+12]="R"; top_ram[0*256+13]=" "; top_ram[0*256+14]="Q"; top_ram[0*256+15]="U";
    top_ram[0*256+16]="A"; top_ram[0*256+17]="L"; top_ram[0*256+18]="I"; top_ram[0*256+19]="T";
    top_ram[0*256+20]="Y"; top_ram[0*256+21]=" "; top_ram[0*256+22]="A"; top_ram[0*256+23]="N";
    top_ram[0*256+24]="A"; top_ram[0*256+25]="L"; top_ram[0*256+26]="Y"; top_ram[0*256+27]="Z";
    top_ram[0*256+28]="E"; top_ram[0*256+29]="R";
    top_ram[0*256+148]="A"; top_ram[0*256+149]="L"; top_ram[0*256+150]="A";
    top_ram[0*256+151]="R"; top_ram[0*256+152]="M";

    // 面板小字：区头（行 2 = panel_ram 行 2，y32..47）
    panel_ram[2*32+1]="M"; panel_ram[2*32+2]="E"; panel_ram[2*32+3]="A"; panel_ram[2*32+4]="S";
    panel_ram[2*32+5]="U"; panel_ram[2*32+6]="R"; panel_ram[2*32+7]="E"; panel_ram[2*32+8]="M";
    panel_ram[2*32+9]="E"; panel_ram[2*32+10]="N"; panel_ram[2*32+11]="T";
    // 大字标签（lbl_ram stride 8）：Ua Ia THDa / Ub Ib THDb / Uc Ic THDc /
    // FREQ UNB PF PTOT QTOT COMP（15 带占满 y64..543）
    lbl_ram[0*8+0]="U"; lbl_ram[0*8+1]="a";
    lbl_ram[1*8+0]="I"; lbl_ram[1*8+1]="a";
    lbl_ram[2*8+0]="T"; lbl_ram[2*8+1]="H"; lbl_ram[2*8+2]="D"; lbl_ram[2*8+3]="a";
    lbl_ram[3*8+0]="U"; lbl_ram[3*8+1]="b";
    lbl_ram[4*8+0]="I"; lbl_ram[4*8+1]="b";
    lbl_ram[5*8+0]="T"; lbl_ram[5*8+1]="H"; lbl_ram[5*8+2]="D"; lbl_ram[5*8+3]="b";
    lbl_ram[6*8+0]="U"; lbl_ram[6*8+1]="c";
    lbl_ram[7*8+0]="I"; lbl_ram[7*8+1]="c";
    lbl_ram[8*8+0]="T"; lbl_ram[8*8+1]="H"; lbl_ram[8*8+2]="D"; lbl_ram[8*8+3]="c";
    lbl_ram[9*8+0]="F"; lbl_ram[9*8+1]="R"; lbl_ram[9*8+2]="E"; lbl_ram[9*8+3]="Q";
    lbl_ram[10*8+0]="U"; lbl_ram[10*8+1]="N"; lbl_ram[10*8+2]="B";
    lbl_ram[11*8+0]="P"; lbl_ram[11*8+1]="F";
    lbl_ram[12*8+0]="P"; lbl_ram[12*8+1]="T"; lbl_ram[12*8+2]="O"; lbl_ram[12*8+3]="T";
    lbl_ram[13*8+0]="Q"; lbl_ram[13*8+1]="T"; lbl_ram[13*8+2]="O"; lbl_ram[13*8+3]="T";
    lbl_ram[14*8+0]="C"; lbl_ram[14*8+1]="O"; lbl_ram[14*8+2]="M"; lbl_ram[14*8+3]="P";
    // 单位（panel 小字右缘，与数值基线行对齐：带 k 数值基线 = panel 行 2k+5）
    panel_ram[5*32+24]="V";  panel_ram[7*32+24]="A";  panel_ram[9*32+24]="%";
    panel_ram[11*32+24]="V"; panel_ram[13*32+24]="A"; panel_ram[15*32+24]="%";
    panel_ram[17*32+24]="V"; panel_ram[19*32+24]="A"; panel_ram[21*32+24]="%";
    panel_ram[23*32+24]="H"; panel_ram[23*32+25]="z";
    panel_ram[25*32+24]="%";
    panel_ram[29*32+24]="W";                            // PTOT（带 12）
    panel_ram[31*32+23]="v"; panel_ram[31*32+24]="a"; panel_ram[31*32+25]="r";  // QTOT（带 13）
    // COMP 单位（带 14，行 33）由 fmt 字段动态写 uF/mH，初始空白
    // 面板底部小字数据区：SEQ 相序 / PH 电压相位差 / VI 功率因数角（数值由 fmt 动态写）
    panel_ram[35*32+0]="S"; panel_ram[35*32+1]="E"; panel_ram[35*32+2]="Q";
    panel_ram[36*32+0]="P"; panel_ram[36*32+1]="H"; panel_ram[36*32+2]=" ";
    panel_ram[36*32+3]="A"; panel_ram[36*32+4]="B";
    panel_ram[36*32+10]="B"; panel_ram[36*32+11]="C";
    panel_ram[36*32+17]="C"; panel_ram[36*32+18]="A";
    panel_ram[37*32+0]="V"; panel_ram[37*32+1]="I"; panel_ram[37*32+2]=" ";
    panel_ram[37*32+3]="A";
    panel_ram[37*32+10]="B";
    panel_ram[37*32+17]="C";
    // 面板说明区（左下脚注带位流版本标识：看到 V28 = 全通道帧流三点中值滤波
    // （坏帧彻底滤除）+ PLL 双帧武装/触发确认 + 直流常数直减 2160）
    panel_ram[40*32+1]="V"; panel_ram[40*32+2]="3"; panel_ram[40*32+3]="8";
    panel_ram[40*32+4]=" ";
    panel_ram[40*32+5]="M"; panel_ram[40*32+6]="L"; panel_ram[40*32+7]="K";
    panel_ram[40*32+8]="P"; panel_ram[40*32+9]="A"; panel_ram[40*32+10]="I";
    panel_ram[40*32+11]=" ";
    panel_ram[40*32+12]="E"; panel_ram[40*32+13]="G";
    panel_ram[40*32+14]="4"; panel_ram[40*32+15]="S"; panel_ram[40*32+16]="2"; panel_ram[40*32+17]="0";

    // 谐波表（mode 2，16×32 大字，表 16 行×64 列 stride）
    // 表头行 0：HARM / IA(%) / B(%) / C(%)（V/I 挡由 fmt 字段换源，表头固定写 IA 标记）
    table_ram[0*64+1]="H"; table_ram[0*64+2]="A"; table_ram[0*64+3]="R"; table_ram[0*64+4]="M";
    table_ram[0*64+7]="I"; table_ram[0*64+8]="A"; table_ram[0*64+9]="("; table_ram[0*64+10]="%"; table_ram[0*64+11]=")";
    table_ram[0*64+17]="B"; table_ram[0*64+18]="("; table_ram[0*64+19]="%"; table_ram[0*64+20]=")";
    table_ram[0*64+27]="C"; table_ram[0*64+28]="("; table_ram[0*64+29]="%"; table_ram[0*64+30]=")";
    for (ii = 1; ii <= 15; ii = ii + 1) begin
        table_ram[ii*64+1] = (ii >= 10) ? "1" : " ";
        table_ram[ii*64+2] = 8'h30 + (ii % 10);
    end
end

// ---------------------------------------------------------------------------
// 5. BCD 格式化状态机
// ---------------------------------------------------------------------------
localparam F_IDLE = 3'd0, F_FETCH = 3'd1, F_LOAD = 3'd2, F_CONV = 3'd3, F_WRITE = 3'd4;
reg [2:0] fmt_st;
reg [6:0] fmt_f;
reg [4:0] fmt_i;
// 谐波表 BRAM 读地址（与旧 harm_px 索引等价：bank={vi_cur,列}, idx=fmt_f[3:0]−7+列；
// 4bit 减法回绕有意为之：23..37/38..52/53..67 的 fmt_f[3:0] 均落到 0..14）
wire [1:0] hc_col = (fmt_f <= 7'd37) ? 2'd0 : (fmt_f <= 7'd52) ? 2'd1 : 2'd2;
wire [3:0] hc_raw = fmt_f[3:0] - 4'd7;
wire [3:0] hc_idx = hc_raw + {2'd0, hc_col};
wire [6:0] harm_fm_adr = {vi_cur, hc_col, hc_idx};
reg [15:0] harm_fm_q;                 // F_FETCH 拍送地址，F_LOAD 拍出数
reg [15:0] fmt_val;
reg [19:0] fmt_acc;
reg fmt_req;
reg [16:0] fmt_init_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) fmt_init_cnt <= 17'd0;
    else if (fmt_init_cnt < 17'h1_0001) fmt_init_cnt <= fmt_init_cnt + 1'b1;
end
wire fmt_fire = meas_upd_ok | mode_change | func_upd | (fmt_init_cnt == 17'h1_0000);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) fmt_req <= 1'b0;
    else if (fmt_st == F_IDLE) fmt_req <= 1'b0;
    else if (fmt_fire) fmt_req <= 1'b1;
end

reg [15:0] cfg_val;
reg [1:0] cfg_pat;      // 0="###.#" 1="#####" 2="±###" 3=特殊
reg [2:0] cfg_ram;      // 0=panel 1=table 2=top 3=val 4=lbl
reg [10:0] cfg_base;
reg [4:0] cfg_wlast;

// 功率区数据（pwr_px 每拍刷新）
wire pw_comp_act = pw_cpres && !pw_pfok;   // 有电流且 PF≠1：显示补偿容量

always @* begin
    cfg_val = 16'd0; cfg_pat = 2'd0; cfg_ram = 3'd0; cfg_base = 11'd0; cfg_wlast = 5'd4;
    if (fmt_f <= 7'd10) begin
        cfg_ram = 3'd3;                          // val_ram stride 8
        case (fmt_f)
            7'd0: begin cfg_val = rms_px[0];   cfg_base = 11'd0;  end
            7'd1: begin cfg_val = rms_i_px[0]; cfg_base = 11'd8;  end
            7'd2: begin cfg_val = thd_px[0];   cfg_base = 11'd16; end
            7'd3: begin cfg_val = rms_px[1];   cfg_base = 11'd24; end
            7'd4: begin cfg_val = rms_i_px[1]; cfg_base = 11'd32; end
            7'd5: begin cfg_val = thd_px[1];   cfg_base = 11'd40; end
            7'd6: begin cfg_val = rms_px[2];   cfg_base = 11'd48; end
            7'd7: begin cfg_val = rms_i_px[2]; cfg_base = 11'd56; end
            7'd8: begin cfg_val = thd_px[2];   cfg_base = 11'd64; end
            7'd9: begin cfg_val = freq_px;     cfg_base = 11'd72; end
            7'd10: begin cfg_val = unb_px;     cfg_base = 11'd80; end
        endcase
    end else if (fmt_f == 7'd11) begin
        cfg_pat = 2'd3; cfg_ram = 3'd3; cfg_base = 11'd88; cfg_wlast = 5'd3;   // PF "0.xx"
    end else if (fmt_f == 7'd12) begin
        cfg_val = pw_ptot_w; cfg_pat = 2'd1; cfg_ram = 3'd3; cfg_base = 11'd96; cfg_wlast = 5'd4;
    end else if (fmt_f == 7'd13) begin
        // QTOT 4 字符（x112..175），给右缘 var 单位（x184..207）留位
        cfg_val = pw_qtot_w; cfg_pat = 2'd1; cfg_ram = 3'd3; cfg_base = 11'd104; cfg_wlast = 5'd3;
    end else if (fmt_f == 7'd14) begin
        // COMP：感性（lag）→并联电容 pw_cap_w µF（####）；容性→串联电感 pw_ind_w 0.1mH（###.#）
        cfg_ram = 3'd3; cfg_base = 11'd112;
        if (pw_comp_act) begin
            if (pw_lag) begin cfg_pat = 2'd1; cfg_val = {16'd0, pw_cap_w}; cfg_wlast = 5'd3; end
            else        begin cfg_pat = 2'd0; cfg_val = {16'd0, pw_ind_w}; cfg_wlast = 5'd4; end
        end else begin
            cfg_pat = 2'd3; cfg_wlast = 5'd4;
        end
    end else if (fmt_f == 7'd15) begin
        cfg_pat = 2'd3; cfg_ram = 3'd0; cfg_base = 11'd1080; cfg_wlast = 5'd1;  // COMP 单位 uF/mH（x192）
    end else if (fmt_f == 7'd16) begin
        cfg_pat = 2'd3; cfg_ram = 3'd0; cfg_base = 11'd1124; cfg_wlast = 5'd2;  // SEQ ABC/ACB
    end else if (fmt_f <= 7'd19) begin
        // PH AB/BC/CA（行 36，±###），cfg_val = {符号, 9bit 角度}（16 位，勿把 16bit pdeg 整个拼入）
        cfg_pat = 2'd2; cfg_ram = 3'd0; cfg_wlast = 5'd3;
        case (fmt_f)
            7'd17: begin cfg_val = {6'd0, pneg_px[0], pdeg_px[0][8:0]}; cfg_base = 11'd1157; end
            7'd18: begin cfg_val = {6'd0, pneg_px[1], pdeg_px[1][8:0]}; cfg_base = 11'd1164; end
            default: begin cfg_val = {6'd0, pneg_px[2], pdeg_px[2][8:0]}; cfg_base = 11'd1171; end
        endcase
    end else if (fmt_f <= 7'd22) begin
        // VI A/B/C（行 37，V→I 功率因数角）
        cfg_pat = 2'd2; cfg_ram = 3'd0; cfg_wlast = 5'd3;
        case (fmt_f)
            7'd20: begin cfg_val = {6'd0, pneg_i_px[0], pdeg_i_px[0][8:0]}; cfg_base = 11'd1189; end
            7'd21: begin cfg_val = {6'd0, pneg_i_px[1], pdeg_i_px[1][8:0]}; cfg_base = 11'd1196; end
            default: begin cfg_val = {6'd0, pneg_i_px[2], pdeg_i_px[2][8:0]}; cfg_base = 11'd1203; end
        endcase
    end else if (fmt_f <= 7'd37) begin
        // 谐波表 A 列（h1..h15，V/I 挡换源；表格行 0 为表头，数值从行 1 起）
        // 值取自 harm_fm_ram 同步读：F_FETCH 拍送地址，F_LOAD 拍 BRAM 出数
        cfg_val = harm_fm_q;
        cfg_ram = 3'd1;
        cfg_base = 11'd6 + ({5'd0, fmt_f - 7'd22} << 6);
    end else if (fmt_f <= 7'd52) begin
        // 谐波表 B 列
        cfg_val = harm_fm_q;
        cfg_ram = 3'd1;
        cfg_base = 11'd16 + ({5'd0, fmt_f - 7'd37} << 6);
    end else if (fmt_f <= 7'd67) begin
        // 谐波表 C 列
        cfg_val = harm_fm_q;
        cfg_ram = 3'd1;
        cfg_base = 11'd26 + ({5'd0, fmt_f - 7'd52} << 6);
    end else if (fmt_f == 7'd68) begin
        cfg_pat = 2'd3; cfg_ram = 3'd2; cfg_base = 11'd35; cfg_wlast = 5'd8;    // 模式名（顶栏胶囊）
    end else begin
        cfg_pat = 2'd3; cfg_ram = 3'd2; cfg_base = 11'd52; cfg_wlast = 5'd8;    // 功能名（顶栏）
    end
end

reg [1:0] cfg_pat_r;
reg [2:0] cfg_ram_r;
reg [10:0] cfg_base_r;
reg [4:0] cfg_wlast_r;

function [19:0] da3;
    input [19:0] v;
    begin
        da3[3:0]   = (v[3:0]   > 4'd4) ? v[3:0]   + 4'd3 : v[3:0];
        da3[7:4]   = (v[7:4]   > 4'd4) ? v[7:4]   + 4'd3 : v[7:4];
        da3[11:8]  = (v[11:8]  > 4'd4) ? v[11:8]  + 4'd3 : v[11:8];
        da3[15:12] = (v[15:12] > 4'd4) ? v[15:12] + 4'd3 : v[15:12];
        da3[19:16] = (v[19:16] > 4'd4) ? v[19:16] + 4'd3 : v[19:16];
    end
endfunction

wire [3:0] d4 = fmt_acc[19:16];
wire [3:0] d3 = fmt_acc[15:12];
wire [3:0] d2 = fmt_acc[11:8];
wire [3:0] d1 = fmt_acc[7:4];
wire [3:0] d0 = fmt_acc[3:0];

reg [8*9-1:0] mode_str;
always @* begin
    case (mode_px)
        2'd0: mode_str = " WAVEFORM";
        2'd1: mode_str = " SPECTRUM";
        default: mode_str = " HARMONIC";
    endcase
end
reg [8*9-1:0] func_str;
always @* begin
    case (func_px[10:9])
        2'd0: func_str = " SELFTEST";
        2'd1: func_str = " 1PH-A   ";
        2'd2: func_str = " 3P-4WIRE";
        default: func_str = " 3P-3WIRE";
    endcase
end

reg [7:0] wchar;
reg fmt_phs_s;            // ±### 符号（F_LOAD 锁存，F_CONV 会移坏 cfg_val）
// V37：测量模式（func_mode≠0）下 THD 大字带字段（THDa/THDb/THDc = fmt_f 2/5/8）
// 用 "##.#"=显示值÷10（用户标定：外部测量 THD 观感偏大）；自检模式（=0）原样
wire cfg_thd10 = (fmode_px != 2'd0) &&
                 ((fmt_f == 7'd2) || (fmt_f == 7'd5) || (fmt_f == 7'd8));
// 角度 0..359 二进制 → 百/十/个位 BCD，三级流水（每拍只有一级乘法/减法；
// 单拍级联 ×41→×100→×205→×10 曾致 75MHz 域 -8.6ns）：
//   F_LOAD: 锁 ph_bin_r
//   F_CONV fmt_i=0: 除 100 → ph_h_r、余数 ph_rem_r
//   F_CONV fmt_i=1: 余数除 10 → ph_t_r、ph_o_r
reg [8:0]  ph_bin_r;
wire [16:0] ph_hx_w   = ph_bin_r * 8'd41;
wire [3:0]  ph_h_w    = ph_hx_w[16:12];
wire [13:0] ph_h100_w = ph_h_w * 10'd100;
wire [9:0]  ph_rem_w  = ph_bin_r - ph_h100_w[9:0];
reg [3:0]  ph_h_r, ph_t_r, ph_o_r;
reg [9:0]  ph_rem_r;
wire [17:0] ph_tx_w   = ph_rem_r * 8'd205;
wire [3:0]  ph_t_w    = ph_tx_w[17:11];
wire [13:0] ph_t10_w  = ph_t_w * 10'd10;
wire [3:0]  ph_o_w    = ph_rem_r - ph_t10_w[13:0];
always @* begin
    wchar = 8'h20;
    case (cfg_pat_r)
        2'd0: begin   // "###.#"；测量模式 THD 字段用 "##.#"=值÷10（V37，自检原样）
            if (cfg_thd10) begin
                case (fmt_i)
                    5'd0: wchar = 8'h20;
                    5'd1: wchar = (d3 != 4'd0) ? (8'h30 + d3) : 8'h20;
                    // V38：前导零——计算值个位数（如 9%→显示 0.9）时补 "0"
                    // 而非空白（用户实测 ".9" 应为 "0.9"）
                    5'd2: wchar = (d3 != 4'd0 || d2 != 4'd0) ? (8'h30 + d2) : 8'h30;
                    5'd3: wchar = 8'h2E;
                    5'd4: wchar = 8'h30 + d1;
                    default: wchar = 8'h20;
                endcase
            end else begin
                case (fmt_i)
                    5'd0: wchar = (d3 != 4'd0) ? (8'h30 + d3) : 8'h20;
                    5'd1: wchar = (d3 != 4'd0 || d2 != 4'd0) ? (8'h30 + d2) : 8'h20;
                    5'd2: wchar = 8'h30 + d1;
                    5'd3: wchar = 8'h2E;
                    5'd4: wchar = 8'h30 + d0;
                    default: wchar = 8'h20;
                endcase
            end
        end
        2'd1: begin   // "#####"；wlast=3 时右对齐 4 位（d3..d0），wlast=4 为 5 位
            case ({2'b00, fmt_i} + (cfg_wlast_r == 5'd3 ? 6'd1 : 6'd0))
                6'd0: wchar = (d4 != 4'd0) ? (8'h30 + d4) : 8'h20;
                6'd1: wchar = (d4 != 4'd0 || d3 != 4'd0) ? (8'h30 + d3) : 8'h20;
                6'd2: wchar = (d4 != 4'd0 || d3 != 4'd0 || d2 != 4'd0) ? (8'h30 + d2) : 8'h20;
                6'd3: wchar = (d4 != 4'd0 || d3 != 4'd0 || d2 != 4'd0 || d1 != 4'd0) ? (8'h30 + d1) : 8'h20;
                6'd4: wchar = 8'h30 + d0;
                default: wchar = 8'h20;
            endcase
        end
        2'd2: begin   // "±###"（符号 + 3 位 BCD 角度）
            case (fmt_i)
                5'd0: wchar = fmt_phs_s ? 8'h2D : 8'h2B;
                5'd1: wchar = 8'h30 + ph_h_r;
                5'd2: wchar = 8'h30 + ph_t_r;
                5'd3: wchar = 8'h30 + ph_o_r;
                default: wchar = 8'h20;
            endcase
        end        2'd3: begin
            if (fmt_f == 7'd11) begin   // PF "0.xx"（无电流 pw_cpres=0 显示 " -- "）
                case (fmt_i)
                    5'd0: wchar = pw_cpres ? "0" : 8'h20;
                    5'd1: wchar = pw_cpres ? "." : 8'h2D;
                    5'd2: wchar = pw_cpres ? (8'h30 + pf_tens_r) : 8'h2D;
                    5'd3: wchar = pw_cpres ? (8'h30 + pf_ones_r) : 8'h20;
                    default: wchar = 8'h20;
                endcase
            end else if (fmt_f == 7'd14) begin   // COMP 未补偿："--"
                wchar = (fmt_i == 5'd1 || fmt_i == 5'd2) ? 8'h2D : 8'h20;
            end else if (fmt_f == 7'd15) begin   // COMP 单位 uF/mH（未补偿时空白）
                if (!pw_comp_act)      wchar = 8'h20;
                else if (fmt_i == 5'd0) wchar = pw_lag ? "u" : "m";
                else if (fmt_i == 5'd1) wchar = pw_lag ? "F" : "H";
                else                   wchar = 8'h20;
            end else if (fmt_f == 7'd16) begin   // 相序 ABC/ACB
                case (fmt_i)
                    5'd0:   wchar = "A";
                    5'd1:   wchar = seq_px ? "B" : "C";
                    5'd2:   wchar = seq_px ? "C" : "B";
                    default: wchar = 8'h20;
                endcase
            end else if (fmt_f == 7'd68) begin
                wchar = mode_str[8*(5'd8 - fmt_i) +: 8];
            end else if (fmt_f == 7'd69) begin
                wchar = func_str[8*(5'd8 - fmt_i) +: 8];
            end else begin
                wchar = 8'h20;
            end
        end
        default: wchar = 8'h20;
    endcase
end

wire [19:0] da3_w = da3(fmt_acc);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fmt_st <= F_IDLE; fmt_f <= 7'd0; fmt_i <= 5'd0;
        fmt_val <= 16'd0; fmt_acc <= 20'd0;
        fmt_phs_s <= 1'b0;
        ph_bin_r <= 9'd0; ph_rem_r <= 10'd0;
        ph_h_r <= 4'd0; ph_t_r <= 4'd0; ph_o_r <= 4'd0;
    end else begin
        case (fmt_st)
            F_IDLE: if (fmt_req || fmt_fire) begin fmt_f <= 7'd0; fmt_st <= F_FETCH; end
            // F_FETCH：fmt_f 已更新为当前字段，本拍向 harm_fm_ram 送地址（非谐波
            // 字段白送一拍，70 字段×1 拍的开销可忽略），下一拍 BRAM 出数供 F_LOAD
            F_FETCH: fmt_st <= F_LOAD;
            F_LOAD: begin
                fmt_val <= cfg_val; fmt_acc <= 20'd0; fmt_i <= 5'd0;
                cfg_pat_r <= cfg_pat; cfg_ram_r <= cfg_ram;
                cfg_base_r <= cfg_base; cfg_wlast_r <= cfg_wlast;
                fmt_phs_s <= cfg_val[9];
                ph_bin_r  <= cfg_val[8:0];      // → F_CONV 拍1 出百位/余数，拍2 出十位/个位
                fmt_st <= F_CONV;
            end
            F_CONV: begin
                fmt_acc <= {da3_w[18:0], fmt_val[15]};
                fmt_val <= {fmt_val[14:0], 1'b0};
                if (fmt_i == 5'd0) begin
                    ph_h_r   <= ph_h_w;
                    ph_rem_r <= ph_rem_w;       // → 下一拍除 10
                end else if (fmt_i == 5'd1) begin
                    ph_t_r <= ph_t_w;
                    ph_o_r <= ph_o_w;
                end
                if (fmt_i == 5'd15) begin fmt_i <= 5'd0; fmt_st <= F_WRITE; end
                else fmt_i <= fmt_i + 1'b1;
            end
            F_WRITE: begin
                case (cfg_ram_r)
                    3'd0: panel_ram[cfg_base_r + {6'd0, fmt_i}] <= wchar;
                    3'd1: table_ram[cfg_base_r + {6'd0, fmt_i}] <= wchar;
                    3'd2: top_ram[cfg_base_r + {6'd0, fmt_i}] <= wchar;
                    3'd3: val_ram[cfg_base_r[6:0] + {2'd0, fmt_i}] <= wchar;
                    default: lbl_ram[cfg_base_r[6:0] + {2'd0, fmt_i}] <= wchar;
                endcase
                if (fmt_i == cfg_wlast_r) begin
                    if (fmt_f == 7'd69) begin fmt_f <= 7'd0; fmt_st <= F_IDLE; end
                    else begin fmt_f <= fmt_f + 7'd1; fmt_st <= F_FETCH; end
                end else fmt_i <= fmt_i + 1'b1;
            end
            default: fmt_st <= F_IDLE;
        endcase
    end
end

// ---------------------------------------------------------------------------
// 6. 文本渲染流水线
// ---------------------------------------------------------------------------
// ---- c1 地址 ----
// 顶栏大字（行 0，32px）
wire in_ttl_d1 = (hp_d1 >= 12'd16)   && (hp_d1 < 12'd496)  && (vp_d1 < 11'd32);
wire in_mod_d1 = (hp_d1 >= 12'd560)  && (hp_d1 < 12'd704)  && (vp_d1 < 11'd32);
wire in_pfn_d1 = (hp_d1 >= 12'd832)  && (hp_d1 < 12'd992)  && (vp_d1 < 11'd32);
wire in_alm_d1 = (hp_d1 >= 12'd1152) && (hp_d1 < 12'd1232) && (vp_d1 < 11'd32);
wire [8:0] top_ad = in_ttl_d1 ? ({3'b0, hp_d1[9:4]} - 9'd1) :
                    in_mod_d1 ? {3'b0, hp_d1[9:4]} :
                    in_pfn_d1 ? {3'b0, hp_d1[9:4]} :
                    in_alm_d1 ? ({3'b0, hp_d1[9:4]} + 9'd76) : 9'd0;
// 面板数值大字（x112..191，5 字符）、标签大字（x16..111，6 字符）、y64..543（15 带）
wire in_pbig_d1 = (hp_d1 >= 12'd112) && (hp_d1 < 12'd192) &&
                  (vp_d1 >= 11'd64) && (vp_d1 < 11'd544);
wire [5:0] prow_big = {1'b0, vp_d1[9:5]} - 6'd2;
wire [5:0] pcol_big = {1'b0, hp_d1[8:4]} - 6'd7;
wire in_plbl_d1 = (hp_d1 >= 12'd16) && (hp_d1 < 12'd112) &&
                  (vp_d1 >= 11'd64) && (vp_d1 < 11'd544);
wire [5:0] pcol_lbl = {1'b0, hp_d1[8:4]} - 6'd1;
// 小字面板
wire [5:0] trow_d1 = vp_d1[9:4];
wire [4:0] tcol_d1 = hp_d1[7:3];
wire [10:0] pnl_ad = {trow_d1, tcol_d1[4:0]};
wire [8:0] val_ad = {prow_big, pcol_big[2:0]};
wire [8:0] lbl_ad = {prow_big, pcol_lbl[2:0]};
// mode 2 谐波表大字（x232..967、y56..575；行 = vp/32-2，列 = hp/16-15）
wire in_tbl_d1 = (mode_px == 2'd2) && (hp_d1 >= 12'd232) && (hp_d1 < 12'd968) &&
                 (vp_d1 >= 11'd56) && (vp_d1 < 11'd584);
wire [5:0] tbl_row_d1 = vp_d1[10:5] - 6'd2;
wire [6:0] tbl_col_d1 = hp_d1[11:4] - 7'd15;
wire [11:0] tbl_big_ad = {tbl_row_d1, tbl_col_d1[5:0]};

reg [7:0] ch_top, ch_pnl, ch_lbl, ch_val, ch_tbl_big;
always @(posedge clk) begin
    ch_tbl_big <= table_ram[tbl_big_ad];
    ch_top <= top_ram[top_ad];
    ch_pnl <= panel_ram[pnl_ad];
    ch_lbl <= lbl_ram[lbl_ad];
    ch_val <= val_ram[val_ad];
end

// ---- c2 区判定 ----
wire in_ttl_c2 = (hp_d2 >= 12'd16)   && (hp_d2 < 12'd496)  && (vp_d2 < 11'd32);
wire in_mod_c2 = (hp_d2 >= 12'd560)  && (hp_d2 < 12'd704)  && (vp_d2 < 11'd32);
wire in_pfn_c2 = (hp_d2 >= 12'd832)  && (hp_d2 < 12'd992)  && (vp_d2 < 11'd32);
wire in_alm_c2 = (hp_d2 >= 12'd1152) && (hp_d2 < 12'd1232) && (vp_d2 < 11'd32);
wire in_pbig_c2 = (hp_d2 >= 12'd112) && (hp_d2 < 12'd192) &&
                  (vp_d2 >= 11'd64) && (vp_d2 < 11'd544) &&
                  !((vp_d2[9:5] == 5'd15) && (hp_d2 >= 12'd176));   // QTOT 带(vp[9:5]=15) 4 字符，让位右缘 var 单位
wire in_plbl_c2 = (hp_d2 >= 12'd16) && (hp_d2 < 12'd112) &&
                  (vp_d2 >= 11'd64) && (vp_d2 < 11'd544);
wire in_run_c2 = (hp_d2 >= 12'd1040) && (hp_d2 < 12'd1104) && (vp_d2 < 11'd32);
wire in_pnl_c2 = (hp_d2 < 12'd208) && (vp_d2 >= 11'd32) && !in_pbig_c2 && !in_plbl_c2;
wire in_tbl_big_c2 = (mode_px == 2'd2) && (hp_d2 >= 12'd232) && (hp_d2 < 12'd968) &&
                    (vp_d2 >= 11'd56) && (vp_d2 < 11'd584);

// RUN/STOP 字符
reg [7:0] run_ch;
always @* begin
    case (hp_d2[9:4] - (frz_s ? 6'd1 : 6'd2))
        6'd0:    run_ch = frz_s ? "S" : "R";
        6'd1:    run_ch = frz_s ? "T" : "U";
        6'd2:    run_ch = frz_s ? "O" : "N";
        6'd3:    run_ch = frz_s ? "P" : " ";
        default: run_ch = " ";
    endcase
end

wire [7:0] char_big = in_run_c2 ? run_ch :
                      in_pbig_c2 ? ch_val :
                      in_plbl_c2 ? ch_lbl :
                      in_tbl_big_c2 ? ch_tbl_big : ch_top;
reg fix_en;
reg [7:0] fix_ch;
reg [23:0] fix_col;
reg [3:0] fix_yo;       // 小字字形区内相对行号（fix 块按区顶回正）
// 非 fix 区域回落到 panel_ram 小字（单位/SEQ/PH/VI/说明区）；
// 必须按 in_pnl_c2 门控——text_bit 全屏优先级最高，不门控会把面板字符别名投影到主区
wire [7:0] char_small = fix_en ? fix_ch : (in_pnl_c2 ? ch_pnl : 8'h20);
wire [10:0] fix_row = fix_en ? {7'd0, fix_yo} : {7'd0, vp_d2[3:0]};
wire [11:0] font2_addr = {char_big[6:0], vp_d2[4:0]};
wire [10:0] font_addr  = {char_small[6:0], fix_row[3:0]};   // 小字：fix 区相对行 / panel 区 16 对齐行
reg [15:0] font2_bits;
reg [7:0] font_bits;
reg [3:0] rx4_s2;
reg [2:0] rx_s2;
reg big_q;
always @(posedge clk) begin
    font2_bits <= font2_rom[font2_addr];
    font_bits  <= font_rom[font_addr];
    rx4_s2 <= hp_d2[3:0];
    rx_s2  <= hp_d2[2:0];
    big_q  <= in_run_c2 | in_ttl_c2 | in_mod_c2 | in_pfn_c2 | in_alm_c2 | in_pbig_c2 | in_plbl_c2 | in_tbl_big_c2;
end
wire text_bit = big_q ? font2_bits[4'd15 - rx4_s2] : font_bits[3'd7 - rx_s2];

// ---- c2 文本颜色 ----
reg [23:0] text_color;
always @* begin
    text_color = TXT_BODY;
    if (in_run_c2)
        text_color = frz_s ? STOP_TXT : RUN_TXT;
    else if (in_ttl_c2)
        text_color = ACCENT;
    else if (in_mod_c2)
        text_color = BG_MAIN;
    else if (in_pfn_c2) begin
        case (func_px[10:9])
            2'd0:    text_color = ACCENT;
            2'd1:    text_color = TXT_B;
            2'd2:    text_color = PILL1_FILL;
            default: text_color = PILL2_FILL;
        endcase
    end else if (in_alm_c2)
        text_color = alm_act ? (blink_on ? ALM_TXT : ALM_DIMT) : ALM_OFFT;
    else if (in_pbig_c2) begin
        // 15 带布局：每相 3 带（U/I/THD），vp[9:5] 2-4=A、5-7=B、8-10=C
        case (vp_d2[9:5])
            5'd2, 5'd3, 5'd4:   text_color = TXT_A;
            5'd5, 5'd6, 5'd7:   text_color = TXT_B;
            5'd8, 5'd9, 5'd10:  text_color = TXT_C;
            default:            text_color = TXT_BRIGHT;
        endcase
    end else if (in_plbl_c2) begin
        case (vp_d2[9:5])
            5'd2, 5'd3, 5'd4:   text_color = LBL_A;
            5'd5, 5'd6, 5'd7:   text_color = LBL_B;
            5'd8, 5'd9, 5'd10:  text_color = LBL_C;
            default:            text_color = 24'h8A93A8;
        endcase
    end else if (in_tbl_big_c2) begin
        if (vp_d2 < 11'd88)
            text_color = TBL_HDRTX;
        else if ((hp_d2 >= 12'd328) && (hp_d2 < 12'd488))
            text_color = vi_cur ? TXT_A : TXT_A;
        else if ((hp_d2 >= 12'd488) && (hp_d2 < 12'd648))
            text_color = TXT_B;
        else if ((hp_d2 >= 12'd648) && (hp_d2 < 12'd808))
            text_color = TXT_C;
        else
            text_color = 24'h7E8798;
    end else if (in_pnl_c2) begin
        case (vp_d2[9:4])
            6'd2:              text_color = ACCENT;      // MEASUREMENT 区头
            6'd35, 6'd36, 6'd37: text_color = 24'hB8C2D4; // SEQ/PH/VI 数据行
            6'd40:             text_color = 24'h525A6C;  // 品牌脚注
            default:           text_color = 24'h7E8798;
        endcase
    end
end
reg [23:0] txt_col_s2;
always @(posedge clk) txt_col_s2 <= text_color;

// ---- c2 波形 RAM 读 ----
// 512 地址 = 整窗：写侧已按时基抽点（2/4/8 周期窗对应 1/2/4 分频存点），
// 读侧恒等映射；1 周期档取 wf_x0>>1（前 256 地址、4px/点）。旧实现
// {2'd0, wf_x0[7:1]} 丢了 wf_x0 的 bit8，地址序被切成 0..127/64..127 拼接错乱
wire [8:0] wf_x0 = (hp_d1 - 12'd216) >> 1;
reg [8:0] wf_idx;
always @* begin
    case (tb_px)
        -3'sd1:  wf_idx = {1'b0, wf_x0[8:1]};
        default: wf_idx = wf_x0;
    endcase
end
wire [8:0] wf_adr = wf_idx + pan_px;

// c2b：mid/量程偏移寄存
reg [10:0] mid0_d, mid1_d, mid2_d;
always @(posedge clk) begin
    mid0_d <= mid_px[0]; mid1_d <= mid_px[1]; mid2_d <= mid_px[2];
    s0_d <= soff_px0; s1_d <= soff_px1; s2_d <= soff_px2;
end

// 刻度→映射常数：off = (code-2048) × M >> (S - gain)，90px = 当前挡半窗码数。
// 电压挡半窗码数（±挡/2 × 40.96 码/V）：2.5V→102.4 … 50V→2048；
// 电流挡（±挡/2 × 102.35 码/A，9.770mA/码 = 20A 满幅设计）：0.25A→12.8 … 20A→2047。
// 20A 挡半窗=2047 码=满幅：自检波形峰恰好贴窗顶 ±20A 标签，与电压路（满幅=50V 峰贴 ±50V 标签）对称。
// gain(ENC4 1x/2x/4x) 减 S 即放大；sc 预饱和 ±92px（2px 余量给霓虹描边）。
function [16:0] msel_v;         // {S[3:0], M[12:0]}
    input [2:0] idx;
    input [1:0] gain;
    reg [12:0] m;
    reg [3:0]  s;
    begin
        case (idx)
            3'd0: begin m = 13'd7200; s = 4'd13; end   // ±2.5V
            3'd1: begin m = 13'd3600; s = 4'd13; end   // ±5V
            3'd2: begin m = 13'd2400; s = 4'd13; end   // ±7.5V
            3'd3: begin m = 13'd1440; s = 4'd13; end   // ±12.5V
            3'd4: begin m = 13'd1028; s = 4'd13; end   // ±17.5V
            3'd5: begin m = 13'd720;  s = 4'd13; end   // ±25V
            3'd6: begin m = 13'd1029; s = 4'd14; end   // ±35V
            default: begin m = 13'd90; s = 4'd11; end  // ±50V
        endcase
        msel_v = {s - {2'b00, gain}, m};
    end
endfunction
function [16:0] msel_i;
    input [2:0] idx;
    input [1:0] gain;
    reg [12:0] m;
    reg [3:0]  s;
    begin
        case (idx)
            3'd0: begin m = 13'd7213; s = 4'd11; end   // ±0.25A
            3'd1: begin m = 13'd7213; s = 4'd12; end   // ±0.5A
            3'd2: begin m = 13'd7213; s = 4'd13; end   // ±1A
            3'd3: begin m = 13'd2885; s = 4'd13; end   // ±2.5A
            3'd4: begin m = 13'd1442; s = 4'd13; end   // ±5A
            3'd5: begin m = 13'd962;  s = 4'd13; end   // ±7.5A
            3'd6: begin m = 13'd577;  s = 4'd13; end   // ±12.5A
            default: begin m = 13'd360; s = 4'd13; end // ±20A
        endcase
        msel_i = {s - {2'b00, gain}, m};
    end
endfunction

// 波形偏置预计算（样点率架构）：每个采样点到达后，用一套 13 拍移位累加乘法器
// 依次计算 6 通道的窗口内偏移 off = clamp((code-2048)×M >> S, ±92)，拼成 48bit 字
// 后单端口写入 512×48 偏移 RAM。像素流水线只做 mid-off 加法——逐像素宽乘法移除。
// 6 通道×15 拍 = 90 周期 ≪ 586 周期的采样间隔。增益/挡位改变后按新刻度写入，
// 整环刷新需 512 采样（约 40ms）。
localparam O_IDLE = 3'd0, O_LOAD = 3'd1, O_MUL = 3'd2, O_WR = 3'd3, O_FLUSH = 3'd4;
reg [2:0]  o_st;
reg [2:0]  o_chan;                   // 0-2=V a/b/c，3-5=I a/b/c
reg [3:0]  o_t;
reg [8:0]  off_wadr;
reg [11:0] samp_r [0:5];
reg signed [12:0] dv_r;
reg [16:0] msel_r;                   // {S[3:0], M[12:0]}
reg signed [26:0] mul_acc;
reg signed [26:0] dv_sh;
reg [47:0] off_word;
// 通道→窗号/V-I 选择：o_chan 0..2=Va/Vb/Vc，3..5=Ia/Ib/Ic（V/I 边界在 >=3）。
// 勿用 o_chan[2] 判 V/I——3=3'b011 的 bit2 是 0，Ia 会误走电压分支且索引越界
// （V11 位流实测：Ia 按 ±2.5V 电压挡绘制 → 饱和成四排竖点）。显式全译码，
// 结构上杜绝越界索引与分支误选
reg [1:0] og_win;
reg       og_isv;
always @* begin
    case (o_chan)
        3'd0: begin og_win = 2'd0; og_isv = 1'b1; end
        3'd1: begin og_win = 2'd1; og_isv = 1'b1; end
        3'd2: begin og_win = 2'd2; og_isv = 1'b1; end
        3'd3: begin og_win = 2'd0; og_isv = 1'b0; end
        3'd4: begin og_win = 2'd1; og_isv = 1'b0; end
        3'd5: begin og_win = 2'd2; og_isv = 1'b0; end
        default: begin og_win = 2'd0; og_isv = 1'b1; end
    endcase
end
reg [2:0]  og_ev, og_eiv;                 // 本窗有效挡位（自动+偏移，V/I 各自）
always @* begin
    case (og_win)
        2'd0:    begin og_ev = ev0;  og_eiv = eiv0; end
        2'd1:    begin og_ev = ev1;  og_eiv = eiv1; end
        default: begin og_ev = ev2;  og_eiv = eiv2; end
    endcase
end
// 乘积 → 饱和 + 移位 → 8bit 偏移
wire [3:0]         o_s     = msel_r[16:13];
wire [26:0]        clamp_c = 27'd92 <<< o_s;
wire signed [26:0] scat_c  = (mul_acc > $signed(clamp_c)) ? $signed(clamp_c) :
                             (mul_acc < -$signed(clamp_c)) ? -$signed(clamp_c) : mul_acc;
wire signed [7:0]  off_w   = scat_c >>> o_s;

(* ram_style = "block" *) reg signed [47:0] off_ram [0:511];
integer ri;
// V35/V36：无信号通道迹线置中——rms_px/rms_i_px 已在锁存拍门控为 0（读数 0.0/
// THD 0.0 同一"无信号"语义）。开路输入=天线拾取：电流在 0.5A 挡是满窗毛刺带，
// 电压 1~2V 拾取在 ±2.5V 挡也近满窗；置中后无信号=平线，有真实信号（V≥4.0V/
// I≥0.5A）迹线完全不受影响。自检 DDS 35.3V/13.4A 不触发门控
wire v_gated = (o_chan == 3'd0 && rms_px[0] == 16'd0) ||
               (o_chan == 3'd1 && rms_px[1] == 16'd0) ||
               (o_chan == 3'd2 && rms_px[2] == 16'd0);
wire i_gated = (o_chan == 3'd3 && rms_i_px[0] == 16'd0) ||
               (o_chan == 3'd4 && rms_i_px[1] == 16'd0) ||
               (o_chan == 3'd5 && rms_i_px[2] == 16'd0);
wire ch_gated = v_gated || i_gated;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_st <= O_IDLE; o_chan <= 3'd0; off_wadr <= 9'd0; o_t <= 4'd0;
    end else if (wf_capture) begin
        samp_r[0] <= wf_va; samp_r[1] <= wf_vb; samp_r[2] <= wf_vc;
        samp_r[3] <= wf_ia; samp_r[4] <= wf_ib; samp_r[5] <= wf_ic;
        off_wadr <= wf_wr;
        o_chan   <= 3'd0;
        o_st     <= O_LOAD;
    end else begin
        case (o_st)
            O_LOAD: begin
                dv_r   <= ch_gated ? 13'sd0 : ($signed({1'b0, samp_r[o_chan]}) - 13'sd2048);
                msel_r <= og_isv ? msel_v(og_ev, 2'd0)
                                 : msel_i(og_eiv, 2'd0);
                mul_acc <= 27'd0;
                dv_sh   <= ch_gated ? 27'sd0 : ($signed({15'd0, samp_r[o_chan]}) - 27'sd2048);
                o_t     <= 4'd0;
                o_st    <= O_MUL;
            end
            O_MUL: begin
                if (msel_r[o_t]) mul_acc <= mul_acc + dv_sh;
                dv_sh <= dv_sh <<< 1;
                o_t   <= o_t + 4'd1;
                if (o_t == 4'd12) o_st <= O_WR;
            end
            O_WR: begin
                off_word[o_chan*8 +: 8] <= off_w;
                o_st   <= (o_chan == 3'd5) ? O_FLUSH : O_LOAD;
                o_chan <= (o_chan == 3'd5) ? 3'd0 : o_chan + 3'd1;
            end
            O_FLUSH: begin
                off_ram[off_wadr] <= off_word;
                o_st <= O_IDLE;
            end
            default: o_st <= O_IDLE;
        endcase
    end
end

reg signed [12:0] wy_raw_va, wy_raw_vb, wy_raw_vc;
reg signed [12:0] wy_raw_ia, wy_raw_ib, wy_raw_ic;
reg signed [47:0] yoff_wq;
always @(posedge clk) begin
    yoff_wq <= off_ram[wf_adr];
    wy_raw_va <= $signed({2'b00, mid0_d}) - $signed(yoff_wq[7:0]);
    wy_raw_vb <= $signed({2'b00, mid1_d}) - $signed(yoff_wq[15:8]);
    wy_raw_vc <= $signed({2'b00, mid2_d}) - $signed(yoff_wq[23:16]);
    wy_raw_ia <= $signed({2'b00, mid0_d}) - $signed(yoff_wq[31:24]);
    wy_raw_ib <= $signed({2'b00, mid1_d}) - $signed(yoff_wq[39:32]);
    wy_raw_ic <= $signed({2'b00, mid2_d}) - $signed(yoff_wq[47:40]);
end

// ---------------------------------------------------------------------------
// 7. c3 几何
// ---------------------------------------------------------------------------
wire [11:0] hx = hp_d3;
wire [10:0] vy = vp_d3;

localparam WFX0 = 216, WFX1 = 1240;
localparam WAY0 = 64,  WAY1 = 244;
localparam WBY0 = 292, WBY1 = 472;
localparam WCY0 = 520, WCY1 = 700;

wire in_wf_x = (hx >= WFX0) && (hx < WFX1);
wire in_wfa = in_wf_x && (vy >= WAY0) && (vy < WAY1);
wire in_wfb = in_wf_x && (vy >= WBY0) && (vy < WBY1);
wire in_wfc = in_wf_x && (vy >= WCY0) && (vy < WCY1);
wire in_wf_any = in_wfa | in_wfb | in_wfc;
wire [11:0] wfx = hx - WFX0;
wire wf_vmain  = in_wf_any && (wfx[6:0] == 7'd0);
wire wf_vgrid  = in_wf_any && (wfx[5:0] == 6'd0);
wire wf_mid    = in_wf_any && (vy == 11'd154 || vy == 11'd382 || vy == 11'd610);
wire wf_hgrid  = in_wf_any &&
                 (vy == 11'd94  || vy == 11'd124 || vy == 11'd184 || vy == 11'd214 ||
                  vy == 11'd322 || vy == 11'd352 || vy == 11'd412 || vy == 11'd442 ||
                  vy == 11'd550 || vy == 11'd580 || vy == 11'd640 || vy == 11'd670);
wire wf_border = in_wf_any && (vy == WAY0 || vy == WAY1-11'd1 ||
                               vy == WBY0 || vy == WBY1-11'd1 ||
                               vy == WCY0 || vy == WCY1-11'd1 ||
                               hx == WFX0 || hx == WFX1-12'd1);
wire wf_accent = in_wf_any && (hx >= WFX0) && (hx < WFX0 + 12'd3);
wire wf_xtick  = (tb_px == 3'sd0) && (vy >= 11'd700) && (vy < 11'd704) &&
                 (hx == 12'd216 || hx == 12'd472 || hx == 12'd728 ||
                  hx == 12'd984 || hx == 12'd1240);

wire show_a_tr = (chan_eff == 2'd0) || (chan_eff == 2'd1);
wire show_b_tr = (chan_eff == 2'd0) || (chan_eff == 2'd2);
wire show_c_tr = (chan_eff == 2'd0) || (chan_eff == 2'd3);
// ---- 连线段渲染：core 覆盖"本列 y 与前列 y 的纵向区间"，陡坡处不再断点 ----
// 前列 y/窗标志在本级多寄一拍取得；前列不在同窗（窗口左缘/行首）退化为单点。
// 512 环地址 = 整周期，地址回绕处相位连续，平移跨越回绕无跳变
// V38 电压迹线线性插值（A 相）：512 样本铺 1024 列=每样本恰 2 列，旧渲染两列
// 画同一值=陡坡 2px 宽台阶折线（过零斜率最大处最显="不丝滑"视觉主体）。改为
// 偶数列画 mid(前样本,本样本)、奇数列画本样本原值——2 倍上采样线性插值：
// mid(n-1,n), y[n], mid(n,n+1)… 单调无回跳，台阶减半。pv 复用为前样本原值
// 载体（奇列画原值 ⇒ 偶列拍 pv 恰=前样本原值）。B/C 相不插值——当前工作方
// 式仅 A 相有信号（B/C 被 10V 门控为平线，插值无视觉收益），三相传插值版
//（+70LUT）在 1.28 档布线马拉松，收敛为 A 相版 +25LUT nonetheless 于已验证
// 可布的 V37 基线；后续接三相时若需 B/C 同款平滑，取 git 存档三相传版并提
// relaxation。电流迹线不插值（门控后常态平线）。半样本群时延 0.35° 无感；
// 测量/采样/PLL 零改动
reg [10:0] wya_pv, wyb_pv, wyc_pv, wia_pv, wib_pv, wic_pv;
reg        wfa_pv, wfb_pv, wfc_pv;
// 列奇偶标志：列号在 hp_d1 级生成（(hp-216) 奇偶=hp 最低位，216 偶），
// 延迟 2 拍对齐 hp_d3 级（wy_raw/hi-lo 所在拍）
reg        wf_odd_q1, wf_odd_q2;
always @(posedge clk) begin
    wia_pv <= wy_raw_ia[10:0]; wib_pv <= wy_raw_ib[10:0]; wic_pv <= wy_raw_ic[10:0];
    wfa_pv <= in_wfa; wfb_pv <= in_wfb; wfc_pv <= in_wfc;
    wf_odd_q1 <= hp_d1[0];
    wf_odd_q2 <= wf_odd_q1;
end
wire [11:0] vam_w = {1'b0, wya_pv} + {1'b0, wy_raw_va[10:0]};
wire [10:0] va_dr = (!wfa_pv || wf_odd_q2) ? wy_raw_va[10:0] : vam_w[11:1];
always @(posedge clk) begin
    wya_pv <= va_dr; wyb_pv <= wy_raw_vb[10:0]; wyc_pv <= wy_raw_vc[10:0];
end
wire [10:0] va_hi = (wfa_pv && (wya_pv > va_dr)) ? wya_pv : va_dr;
wire [10:0] va_lo = (wfa_pv && (wya_pv < va_dr)) ? wya_pv : va_dr;
wire [10:0] vb_hi = (wfb_pv && (wyb_pv > wy_raw_vb[10:0])) ? wyb_pv : wy_raw_vb[10:0];
wire [10:0] vb_lo = (wfb_pv && (wyb_pv < wy_raw_vb[10:0])) ? wyb_pv : wy_raw_vb[10:0];
wire [10:0] vc_hi = (wfc_pv && (wyc_pv > wy_raw_vc[10:0])) ? wyc_pv : wy_raw_vc[10:0];
wire [10:0] vc_lo = (wfc_pv && (wyc_pv < wy_raw_vc[10:0])) ? wyc_pv : wy_raw_vc[10:0];
wire [10:0] ia_hi = (wfa_pv && (wia_pv > wy_raw_ia[10:0])) ? wia_pv : wy_raw_ia[10:0];
wire [10:0] ia_lo = (wfa_pv && (wia_pv < wy_raw_ia[10:0])) ? wia_pv : wy_raw_ia[10:0];
wire [10:0] ib_hi = (wfb_pv && (wib_pv > wy_raw_ib[10:0])) ? wib_pv : wy_raw_ib[10:0];
wire [10:0] ib_lo = (wfb_pv && (wib_pv < wy_raw_ib[10:0])) ? wib_pv : wy_raw_ib[10:0];
wire [10:0] ic_hi = (wfc_pv && (wic_pv > wy_raw_ic[10:0])) ? wic_pv : wy_raw_ic[10:0];
wire [10:0] ic_lo = (wfc_pv && (wic_pv < wy_raw_ic[10:0])) ? wic_pv : wy_raw_ic[10:0];
// V 迹线受 show_v、I 迹线受 show_i 门控；V core=区间亮线，glow=区间 ±1 辉光
wire core_va = show_a_tr && show_v && in_wfa && (vy >= va_lo) && (vy <= va_hi);
wire core_vb = show_b_tr && show_v && in_wfb && (vy >= vb_lo) && (vy <= vb_hi);
wire core_vc = show_c_tr && show_v && in_wfc && (vy >= vc_lo) && (vy <= vc_hi);
// I 迹线 3px 亮琥珀（区间 ±1）
wire core_ia = show_a_tr && show_i && in_wfa &&
               ((vy + 11'd1 >= ia_lo) && (vy <= ia_hi + 11'd1));
wire core_ib = show_b_tr && show_i && in_wfb &&
               ((vy + 11'd1 >= ib_lo) && (vy <= ib_hi + 11'd1));
wire core_ic = show_c_tr && show_i && in_wfc &&
               ((vy + 11'd1 >= ic_lo) && (vy <= ic_hi + 11'd1));
wire core_v = core_va | core_vb | core_vc;
wire core_i = core_ia | core_ib | core_ic;
wire draw_va = show_a_tr && show_v && in_wfa && !core_va &&
               ((vy + 11'd1 >= va_lo) && (vy <= va_hi + 11'd1));
wire draw_vb = show_b_tr && show_v && in_wfb && !core_vb &&
               ((vy + 11'd1 >= vb_lo) && (vy <= vb_hi + 11'd1));
wire draw_vc = show_c_tr && show_v && in_wfc && !core_vc &&
               ((vy + 11'd1 >= vc_lo) && (vy <= vc_hi + 11'd1));
wire draw_v = draw_va | draw_vb | draw_vc;

// 字符牌（x300..347，窗顶对齐；右移避让左缘电压刻度）
wire chip_a = (hx >= 12'd300) && (hx < 12'd316) && (vy >= 11'd64)  && (vy < 11'd80);
wire chip_b = (hx >= 12'd300) && (hx < 12'd316) && (vy >= 11'd292) && (vy < 11'd308);
wire chip_c = (hx >= 12'd300) && (hx < 12'd316) && (vy >= 11'd520) && (vy < 11'd536);
wire vchip_a = (hx >= 12'd316) && (hx < 12'd332) && (vy >= 11'd64)  && (vy < 11'd80);
wire vchip_b = (hx >= 12'd316) && (hx < 12'd332) && (vy >= 11'd292) && (vy < 11'd308);
wire vchip_c = (hx >= 12'd316) && (hx < 12'd332) && (vy >= 11'd520) && (vy < 11'd536);
wire ichip_a = (hx >= 12'd332) && (hx < 12'd348) && (vy >= 11'd64)  && (vy < 11'd80);
wire ichip_b = (hx >= 12'd332) && (hx < 12'd348) && (vy >= 11'd292) && (vy < 11'd308);
wire ichip_c = (hx >= 12'd332) && (hx < 12'd348) && (vy >= 11'd520) && (vy < 11'd536);
wire mk_a = in_wf_x && (pick_px == 2'd1) && (vy >= WAY1) && (vy < WAY1 + 11'd2);
wire mk_b = in_wf_x && (pick_px == 2'd2) && (vy >= WBY1) && (vy < WBY1 + 11'd2);
wire mk_c = in_wf_x && (pick_px == 2'd3) && (vy >= WCY1) && (vy < WCY1 + 11'd2);

// 顶栏徽章
wire run_fill = (hx >= 12'd1024) && (hx <= 12'd1120) && (vy >= 11'd2) && (vy <= 11'd29);
wire run_edge = (hx == 12'd1024) || (hx == 12'd1120) || (vy == 11'd2) || (vy == 11'd29);
wire alm_fill = (hx >= 12'd1140) && (hx <= 12'd1244) && (vy >= 11'd2) && (vy <= 11'd29);
wire alm_edge = (hx == 12'd1140) || (hx == 12'd1244) || (vy == 11'd2) || (vy == 11'd29);
wire pil_fill = (hx >= 12'd548)  && (hx <= 12'd715)  && (vy >= 11'd2) && (vy <= 11'd29);
wire pil_edge = (hx == 12'd548)  || (hx == 12'd715)  || (vy == 11'd2) || (vy == 11'd29);
reg [23:0] mode_fill, mode_bord;
always @* begin
    case (mode_px)
        2'd0: begin mode_fill = PILL0_FILL; mode_bord = PILL0_BORD; end
        2'd1: begin mode_fill = PILL1_FILL; mode_bord = PILL1_BORD; end
        default: begin mode_fill = PILL2_FILL; mode_bord = PILL2_BORD; end
    endcase
end

// 频谱
localparam SPX0 = 216, SPX1 = 1176;
localparam SPY0 = 72,  SPY1 = 664;
wire [9:0] sp_off_d1 = hp_d1 - SPX0;
wire [3:0] sp_g_d1   = sp_off_d1[9:6];
wire [5:0] sp_r_d1   = sp_off_d1[5:0];
wire sp_ok_d1 = (hp_d1 >= SPX0) && (hp_d1 < SPX1) && (sp_r_d1 < 6'd48) && (sp_g_d1 < 4'd15);
wire [1:0] sp_bar_g_d1 = sp_ok_d1 ? sp_r_d1[5:4] : 2'd0;
wire [3:0] sp_g_g_d1   = sp_ok_d1 ? sp_g_d1 : 4'd0;
reg [1:0] sp_bar_g_q1, sp_bar_g_q2;
reg sp_ok_q1, sp_ok_q2;
// 双谐波 BRAM（TD RAM 推断器对结构挑剔：每 RAM 独立 always、仅一同步读+一
// 条件写、写数据用常量切片 case 选择——变基 part-select 与多阵列同块均会令
// elaborate 阶段 Signal Abort）。频谱读地址 c1 组合送入，BRAM 输出寄存拍位
// 与旧 sp_pct_q 一致，下游 sp_pct_s/sp_hm/sp_top_q 不动。
reg [15:0] harm_sp_q;
reg       hw_run;
reg [2:0] hw_grp;                     // 0..5 = Va/Vb/Vc/Ia/Ib/Ic
reg [3:0] hw_idx;                     // 0..14 谐波序号
function [15:0] hpick;                // 240bit 谐波总线 → 16bit 常量切片
    input [239:0] bus;
    input [3:0] idx;
    begin
        case (idx)
            4'd0:  hpick = bus[15:0];    4'd1:  hpick = bus[31:16];
            4'd2:  hpick = bus[47:32];   4'd3:  hpick = bus[63:48];
            4'd4:  hpick = bus[79:64];   4'd5:  hpick = bus[95:80];
            4'd6:  hpick = bus[111:96];  4'd7:  hpick = bus[127:112];
            4'd8:  hpick = bus[143:128]; 4'd9:  hpick = bus[159:144];
            4'd10: hpick = bus[175:160]; 4'd11: hpick = bus[191:176];
            4'd12: hpick = bus[207:192]; 4'd13: hpick = bus[223:208];
            4'd14: hpick = bus[239:224];
            default: hpick = 16'd0;
        endcase
    end
endfunction
reg [15:0] hw_dat;
always @* begin
    case (hw_grp)
        3'd0:    hw_dat = hpick(harm_pct_va, hw_idx);
        3'd1:    hw_dat = hpick(harm_pct_vb, hw_idx);
        3'd2:    hw_dat = hpick(harm_pct_vc, hw_idx);
        3'd3:    hw_dat = hpick(harm_pct_ia, hw_idx);
        3'd4:    hw_dat = hpick(harm_pct_ib, hw_idx);
        default: hw_dat = hpick(harm_pct_ic, hw_idx);
    endcase
end
// V37：无信号通道谐波页写 0（用户实测 B/C 谐波乱闪的根源——频谱/谐波表此前
// 无任何门控，开路拾取的逐次谐波噪声柱逐轮刷新）。频谱柱与谐波表共用两块
// BRAM，写侧一处门控全页面生效。实现：meas_upd_ok 拍锁存 6bit 掩码（判据与
// 读数门控同源：<10V / <0.5A），写 FSM 只做一位选择——比较器留在 rms 寄存器
// 邻域，仅 6 根短线进谐波 BRAM 写区（首版 6×16bit 比较树直挂写区=96 根长线
// 穿过 91% 占用的拥塞区，1.25/1.28 档布线马拉松的直接诱因）
reg [5:0] hw_gmsk;
always @(posedge clk) begin
    if (meas_upd_ok)
        hw_gmsk <= {(rms_ic < 32'd5),         (rms_ib < 32'd5),         (rms_ia < 32'd5),
                    (rms_vc < V_OPEN_RMS_TH),  (rms_vb < V_OPEN_RMS_TH), (rms_va < V_OPEN_RMS_TH)};
end
wire hw_gated = hw_gmsk[hw_grp];
wire [15:0] hw_dat_g = hw_gated ? 16'd0 : hw_dat;
// 串行写计数：meas_upd_ok 起动，90 拍写完（输入总线在两轮测量间稳定）
always @(posedge clk) begin
    if (meas_upd_ok && !hw_run) begin
        hw_run <= 1'b1; hw_grp <= 3'd0; hw_idx <= 4'd0;
    end else if (hw_run) begin
        if (hw_idx == 4'd14) begin
            hw_idx <= 4'd0;
            if (hw_grp == 3'd5) hw_run <= 1'b0;
            else hw_grp <= hw_grp + 3'd1;
        end else hw_idx <= hw_idx + 4'd1;
    end
end
wire       hw_we  = hw_run;
wire [6:0] hw_adr = {hw_grp, hw_idx};
always @(posedge clk) begin
    harm_sp_q <= harm_sp_ram[{1'b0, sp_bar_g_d1, sp_g_g_d1}];
    if (hw_we) harm_sp_ram[hw_adr] <= hw_dat_g;
end
always @(posedge clk) begin
    harm_fm_q <= harm_fm_ram[harm_fm_adr];
    if (hw_we) harm_fm_ram[hw_adr] <= hw_dat_g;
end
wire [15:0] sp_pct_q = harm_sp_q;
always @(posedge clk) begin
    sp_bar_g_q1 <= sp_bar_g_d1;
    sp_ok_q1 <= sp_ok_d1;
    sp_bar_g_q2 <= sp_bar_g_q1;
    sp_ok_q2 <= sp_ok_q1;
end
wire [15:0] sp_pct_s = (sp_pct_q > 16'd1023) ? 16'd1023 : sp_pct_q;
wire [25:0] sp_hm = sp_pct_s * 16'd576;
reg [10:0] sp_top_q;
always @(posedge clk) sp_top_q <= 11'd663 - {1'b0, sp_hm[19:10]};
wire draw_sp = sp_ok_q2 && (vy >= sp_top_q) && (vy <= 11'd662);
wire sp_cap = draw_sp && (vy < sp_top_q + 11'd5);
wire in_sp_c3 = (hx >= SPX0) && (hx < SPX1) && (vy >= SPY0) && (vy < SPY1);
wire [9:0] sp_off = hx - SPX0;
wire sp_base = (vy == 11'd663 || vy == 11'd664) && (hx >= SPX0) && (hx < SPX1);
wire sp_grid = in_sp_c3 && (sp_off[7:0] == 8'd0);
wire sp_yaxis = (hx == 12'd1180) && (vy >= 11'd72) && (vy <= 11'd664);
wire sp_ytick = (hx >= 12'd1176) && (hx < 12'd1180) &&
                (vy == 11'd663 || vy == 11'd551 || vy == 11'd438 ||
                 vy == 11'd326 || vy == 11'd213 || vy == 11'd101);
wire sp_hgrid = in_sp_c3 && (vy == 11'd551 || vy == 11'd438 || vy == 11'd326 ||
                             vy == 11'd213 || vy == 11'd101);
wire lc_a = (hx >= 12'd236) && (hx < 12'd252) && (vy >= 11'd48) && (vy < 11'd64);
wire lc_b = (hx >= 12'd260) && (hx < 12'd276) && (vy >= 11'd48) && (vy < 11'd64);
wire lc_c = (hx >= 12'd284) && (hx < 12'd300) && (vy >= 11'd48) && (vy < 11'd64);
wire lc_i = (hx >= 12'd308) && (hx < 12'd324) && (vy >= 11'd48) && (vy < 11'd64);

// 谐波表大字面板（mode 2 几何）
wire tbl_rect   = (hx >= 12'd232) && (hx <= 12'd807) && (vy >= 11'd56) && (vy <= 11'd575);
wire tbl_border = tbl_rect && (hx == 12'd232 || hx == 12'd807 || vy == 11'd56 || vy == 11'd575);
wire tbl_hdrbg  = (vy >= 11'd56) && (vy < 11'd88) && (hx >= 12'd233) && (hx <= 12'd806);
wire tbl_hline  = (vy >= 11'd88) && (vy < 11'd575) && (vy[4:0] == 5'd23) &&
                  (hx >= 12'd233) && (hx < 12'd807);
wire tbl_drow   = (vy >= 11'd88) && (vy < 11'd575);
wire tbl_tint_a = (hx >= 12'd328) && (hx < 12'd488);
wire tbl_tint_b = (hx >= 12'd488) && (hx < 12'd648);
wire tbl_tint_c = (hx >= 12'd648) && (hx < 12'd807);

// 小字 fix 字符（c2）。fix_yo = 字形区内相对行号：小字库按 {字符, 行号} 取模，
// 旧实现直接采样 vp[3:0]，要求字符区 16px 对齐否则字形上下回卷；
// 现由各分支按区顶回正，字符区可放在任意 y。
wire sp_ytk_hit = ((vp_d2 >= 11'd96)  && (vp_d2 < 11'd112)) ||
                  ((vp_d2 >= 11'd208) && (vp_d2 < 11'd224)) ||
                  ((vp_d2 >= 11'd320) && (vp_d2 < 11'd336)) ||
                  ((vp_d2 >= 11'd432) && (vp_d2 < 11'd448)) ||
                  ((vp_d2 >= 11'd544) && (vp_d2 < 11'd560)) ||
                  ((vp_d2 >= 11'd656) && (vp_d2 < 11'd672));
wire [11:0] xl_off = hp_d2 - 12'd216;

// 自适应刻度标签：5 行 = ±半窗 / ±半窗÷3 / 0；数值随挡位(idx)与 ENC4 增益变化。
// 结构 = 符号位 + 4 个数字字符 + 单位；数字查 16 档半窗值表（电压 V / 电流 A 两套）。
// 电压半窗值：0.625 1.25 1.875 2.5 3.125 3.75 4.375 5 6.25 7.5 8.75 12.5 17.5 25 35 50
// 电流半窗值：0.0625 0.125 0.25 0.5 1 2.5 5 7.5 10 12.5 20
function [3:0] hid_v;            // {挡位,增益} → 半窗值编号
    input [4:0] fsi;
    case (fsi)
        5'd0:  hid_v = 4'd3;     // 2.5V
        5'd1:  hid_v = 4'd1;     // 1.25
        5'd2:  hid_v = 4'd0;     // 0.625
        5'd4:  hid_v = 4'd7;     // 5V
        5'd5:  hid_v = 4'd3;
        5'd6:  hid_v = 4'd1;
        5'd8:  hid_v = 4'd9;     // 7.5V
        5'd9:  hid_v = 4'd5;     // 3.75
        5'd10: hid_v = 4'd2;     // 1.875
        5'd12: hid_v = 4'd11;    // 12.5V
        5'd13: hid_v = 4'd8;     // 6.25
        5'd14: hid_v = 4'd4;     // 3.125
        5'd16: hid_v = 4'd12;    // 17.5V
        5'd17: hid_v = 4'd10;    // 8.75
        5'd18: hid_v = 4'd6;     // 4.375
        5'd20: hid_v = 4'd13;    // 25V
        5'd21: hid_v = 4'd11;
        5'd22: hid_v = 4'd8;
        5'd24: hid_v = 4'd14;    // 35V
        5'd25: hid_v = 4'd12;
        5'd26: hid_v = 4'd10;
        5'd28: hid_v = 4'd15;    // 50V
        5'd29: hid_v = 4'd13;
        5'd30: hid_v = 4'd11;
        default: hid_v = 4'd15;
    endcase
endfunction
function [3:0] hid_i;
    input [4:0] fsi;
    case (fsi)
        5'd0:  hid_i = 4'd2;     // 0.25A
        5'd1:  hid_i = 4'd1;     // 0.125
        5'd2:  hid_i = 4'd0;     // 0.0625
        5'd4:  hid_i = 4'd3;     // 0.5A
        5'd5:  hid_i = 4'd2;
        5'd6:  hid_i = 4'd1;
        5'd8:  hid_i = 4'd4;     // 1A
        5'd9:  hid_i = 4'd3;
        5'd10: hid_i = 4'd2;
        5'd12: hid_i = 4'd5;     // 2.5A
        5'd13: hid_i = 4'd4;
        5'd14: hid_i = 4'd3;
        5'd16: hid_i = 4'd6;     // 5A
        5'd17: hid_i = 4'd5;
        5'd18: hid_i = 4'd4;
        5'd20: hid_i = 4'd7;     // 7.5A
        5'd21: hid_i = 4'd13;    // 3.75
        5'd22: hid_i = 4'd11;    // 1.875
        5'd24: hid_i = 4'd9;     // 12.5A
        5'd25: hid_i = 4'd14;    // 6.25
        5'd26: hid_i = 4'd12;    // 3.125
        5'd28: hid_i = 4'd10;    // 20A
        5'd29: hid_i = 4'd8;
        5'd30: hid_i = 4'd6;
        default: hid_i = 4'd10;
    endcase
endfunction
// 半窗值 → BCD 压缩编码（省 LUT）：{clsT,aT,bT,cT, clsM,aM,bM,cM}
// cls0="a.bc"(1≤v<10) cls1="ab.c"(v≥10) cls2="0.bc"(v<1)；a/b/c 为 BCD 数位
function [27:0] sdig_v;
    input [3:0] id;
    case (id)
        4'd0:  sdig_v = {2'd2, 4'd0, 4'd6, 4'd3,  2'd2, 4'd0, 4'd2, 4'd1};  // 0.63/0.21
        4'd1:  sdig_v = {2'd0, 4'd1, 4'd2, 4'd5,  2'd2, 4'd0, 4'd4, 4'd2};  // 1.25/0.42
        4'd2:  sdig_v = {2'd0, 4'd1, 4'd8, 4'd8,  2'd2, 4'd0, 4'd6, 4'd3};  // 1.88/0.63
        4'd3:  sdig_v = {2'd0, 4'd2, 4'd5, 4'd0,  2'd2, 4'd0, 4'd8, 4'd3};  // 2.50/0.83
        4'd4:  sdig_v = {2'd0, 4'd3, 4'd1, 4'd3,  2'd0, 4'd1, 4'd0, 4'd4};  // 3.13/1.04
        4'd5:  sdig_v = {2'd0, 4'd3, 4'd7, 4'd5,  2'd0, 4'd1, 4'd2, 4'd5};  // 3.75/1.25
        4'd6:  sdig_v = {2'd0, 4'd4, 4'd3, 4'd8,  2'd0, 4'd1, 4'd4, 4'd6};  // 4.38/1.46
        4'd7:  sdig_v = {2'd0, 4'd5, 4'd0, 4'd0,  2'd0, 4'd1, 4'd6, 4'd7};  // 5.00/1.67
        4'd8:  sdig_v = {2'd0, 4'd6, 4'd2, 4'd5,  2'd0, 4'd2, 4'd0, 4'd8};  // 6.25/2.08
        4'd9:  sdig_v = {2'd0, 4'd7, 4'd5, 4'd0,  2'd0, 4'd2, 4'd5, 4'd0};  // 7.50/2.50
        4'd10: sdig_v = {2'd0, 4'd8, 4'd7, 4'd5,  2'd0, 4'd2, 4'd9, 4'd2};  // 8.75/2.92
        4'd11: sdig_v = {2'd1, 4'd1, 4'd2, 4'd5,  2'd0, 4'd4, 4'd1, 4'd7};  // 12.5/4.17
        4'd12: sdig_v = {2'd1, 4'd1, 4'd7, 4'd5,  2'd0, 4'd5, 4'd8, 4'd3};  // 17.5/5.83
        4'd13: sdig_v = {2'd1, 4'd2, 4'd5, 4'd0,  2'd0, 4'd8, 4'd3, 4'd3};  // 25.0/8.33
        4'd14: sdig_v = {2'd1, 4'd3, 4'd5, 4'd0,  2'd1, 4'd1, 4'd1, 4'd7};  // 35.0/11.7
        default: sdig_v = {2'd1, 4'd5, 4'd0, 4'd0,  2'd1, 4'd1, 4'd6, 4'd7}; // 50.0/16.7
    endcase
endfunction
function [27:0] sdig_i;
    input [3:0] id;
    case (id)
        4'd0:  sdig_i = {2'd2, 4'd0, 4'd0, 4'd6,  2'd2, 4'd0, 4'd0, 4'd2};  // 0.06/0.02
        4'd1:  sdig_i = {2'd2, 4'd0, 4'd1, 4'd3,  2'd2, 4'd0, 4'd0, 4'd4};  // 0.13/0.04
        4'd2:  sdig_i = {2'd2, 4'd0, 4'd2, 4'd5,  2'd2, 4'd0, 4'd0, 4'd8};  // 0.25/0.08
        4'd3:  sdig_i = {2'd2, 4'd0, 4'd5, 4'd0,  2'd2, 4'd0, 4'd1, 4'd7};  // 0.50/0.17
        4'd4:  sdig_i = {2'd0, 4'd1, 4'd0, 4'd0,  2'd2, 4'd0, 4'd3, 4'd3};  // 1.00/0.33
        4'd5:  sdig_i = {2'd0, 4'd2, 4'd5, 4'd0,  2'd2, 4'd0, 4'd8, 4'd3};  // 2.50/0.83
        4'd6:  sdig_i = {2'd0, 4'd5, 4'd0, 4'd0,  2'd0, 4'd1, 4'd6, 4'd7};  // 5.00/1.67
        4'd7:  sdig_i = {2'd0, 4'd7, 4'd5, 4'd0,  2'd0, 4'd2, 4'd5, 4'd0};  // 7.50/2.50
        4'd8:  sdig_i = {2'd1, 4'd1, 4'd0, 4'd0,  2'd0, 4'd3, 4'd3, 4'd3};  // 10.0/3.33
        4'd9:  sdig_i = {2'd1, 4'd1, 4'd2, 4'd5,  2'd0, 4'd4, 4'd1, 4'd7};  // 12.5/4.17
        4'd10: sdig_i = {2'd1, 4'd2, 4'd0, 4'd0,  2'd0, 4'd6, 4'd6, 4'd7};  // 20.0/6.67
        4'd11: sdig_i = {2'd0, 4'd1, 4'd8, 4'd8,  2'd2, 4'd0, 4'd6, 4'd3};  // 1.88/0.63
        4'd12: sdig_i = {2'd0, 4'd3, 4'd1, 4'd3,  2'd0, 4'd1, 4'd0, 4'd4};  // 3.13/1.04
        4'd13: sdig_i = {2'd0, 4'd3, 4'd7, 4'd5,  2'd0, 4'd1, 4'd2, 4'd5};  // 3.75/1.25
        4'd14: sdig_i = {2'd0, 4'd6, 4'd2, 4'd5,  2'd0, 4'd2, 4'd0, 4'd8};  // 6.25/2.08
        default: sdig_i = {2'd2, 4'd0, 4'd0, 4'd0,  2'd2, 4'd0, 4'd0, 4'd0};
    endcase
endfunction
// 行/列 → 字符：col0=符号 col5=单位，中间 4 字符由 cls 模式与 BCD 数位装配
function [7:0] scl_fmt;
    input [27:0] sd;
    input [2:0]  row;    // 0=+半窗 1=+1/3 2=0 3=-1/3 4=-半窗
    input [2:0]  col;    // 0..5
    input        kind;   // 0=V 1=A
    reg [1:0] cls;
    reg [3:0] da, db, dc;
    reg [7:0] m1, m2, m3, m4, sgc;
    begin
        if (row == 3'd0 || row == 3'd4) begin
            cls = sd[27:26]; {da, db, dc} = sd[25:14];
        end else if (row == 3'd1 || row == 3'd3) begin
            cls = sd[13:12]; {da, db, dc} = sd[11:0];
        end else begin
            cls = 2'd3; da = 4'd0; db = 4'd0; dc = 4'd0;
        end
        case (cls)
            2'd0: begin m1 = 8'h30 + {4'd0, da}; m2 = 8'h2E; m3 = 8'h30 + {4'd0, db}; m4 = 8'h30 + {4'd0, dc}; end
            2'd1: begin m1 = 8'h30 + {4'd0, da}; m2 = 8'h30 + {4'd0, db}; m3 = 8'h2E; m4 = 8'h30 + {4'd0, dc}; end
            2'd2: begin m1 = 8'h30;              m2 = 8'h2E; m3 = 8'h30 + {4'd0, db}; m4 = 8'h30 + {4'd0, dc}; end
            default: begin m1 = 8'h20; m2 = 8'h20; m3 = 8'h20; m4 = 8'h30; end
        endcase
        sgc = (row >= 3'd3) ? 8'h2D : 8'h20;
        case (col)
            3'd0:    scl_fmt = sgc;
            3'd1:    scl_fmt = m1;
            3'd2:    scl_fmt = m2;
            3'd3:    scl_fmt = m3;
            3'd4:    scl_fmt = m4;
            default: scl_fmt = kind ? 8'h41 : 8'h56;
        endcase
    end
endfunction

// 刻度行解码：三窗（顶 64/292/520）内相对偏移 0/48/80/128/160 → 行 0..4，其余无刻度
wire [10:0] wf_yoff = (vp_d2 < 11'd64)  ? 11'd255 :
                      (vp_d2 < 11'd256) ? (vp_d2 - 11'd64)  :
                      (vp_d2 < 11'd292) ? 11'd255 :
                      (vp_d2 < 11'd484) ? (vp_d2 - 11'd292) :
                      (vp_d2 < 11'd520) ? 11'd255 :
                      (vp_d2 < 11'd712) ? (vp_d2 - 11'd520) :
                      11'd255;
wire [2:0] scl_row = (wf_yoff < 11'd16)  ? 3'd0 :
                     (wf_yoff >= 11'd48  && wf_yoff < 11'd64)  ? 3'd1 :
                     (wf_yoff >= 11'd80  && wf_yoff < 11'd96)  ? 3'd2 :
                     (wf_yoff >= 11'd128 && wf_yoff < 11'd144) ? 3'd3 :
                     (wf_yoff >= 11'd160 && wf_yoff < 11'd176) ? 3'd4 : 3'd7;
// 窗选择 + 当前窗的挡位/增益 → 标签值
wire [1:0] win_sel = (vp_d2 >= 11'd64)  && (vp_d2 < 11'd256) ? 2'd0 :
                     (vp_d2 >= 11'd292) && (vp_d2 < 11'd484) ? 2'd1 :
                     (vp_d2 >= 11'd520) && (vp_d2 < 11'd712) ? 2'd2 : 2'd3;
wire [1:0] win_g = 2'd0;                  // 增益维度已由 ENC4 手动量程取代，恒取 0 行
wire [2:0] widx_v = (win_sel == 2'd0) ? ev0 :
                    (win_sel == 2'd1) ? ev1 : ev2;
wire [2:0] widx_i = (win_sel == 2'd0) ? eiv0 :
                    (win_sel == 2'd1) ? eiv1 : eiv2;
wire [3:0] hid_v_w = hid_v({widx_v, win_g});
wire [3:0] hid_i_w = hid_i({widx_i, win_g});
wire [27:0] sd_v = sdig_v(hid_v_w);
wire [27:0] sd_i = sdig_i(hid_i_w);
wire [11:0] sl_off = hp_d2 - 12'd217;    // 左缘标签列基准
wire [11:0] sr_off = hp_d2 - 12'd1184;   // 右缘标签列基准
// 字符牌窗顶低 4 位（字形行回正）：A 窗 64 / B 窗 292 / C 窗 520
wire [3:0] chip_toff = (vp_d2 < 11'd256) ? 4'd0 : (vp_d2 < 11'd484) ? 4'd4 : 4'd8;

always @* begin
    fix_en = 1'b0; fix_ch = 8'h20; fix_col = TXT_BODY; fix_yo = 4'd0;
    if (mode_px == 2'd0) begin
        // 相色牌字母 + V/I 牌（x300..347，右移避让左缘刻度）
        if (hp_d2 >= 12'd304 && hp_d2 < 12'd312 && vp_d2 >= 11'd64 && vp_d2 < 11'd80) begin
            fix_en = 1'b1; fix_ch = "A"; fix_col = 24'h2A0D0D; fix_yo = vp_d2[3:0] - chip_toff;
        end else if (hp_d2 >= 12'd304 && hp_d2 < 12'd312 && vp_d2 >= 11'd292 && vp_d2 < 11'd308) begin
            fix_en = 1'b1; fix_ch = "B"; fix_col = 24'h0A2A14; fix_yo = vp_d2[3:0] - chip_toff;
        end else if (hp_d2 >= 12'd304 && hp_d2 < 12'd312 && vp_d2 >= 11'd520 && vp_d2 < 11'd536) begin
            fix_en = 1'b1; fix_ch = "C"; fix_col = 24'h0D1230; fix_yo = vp_d2[3:0] - chip_toff;
        end else if (hp_d2 >= 12'd320 && hp_d2 < 12'd328 &&
                     (vp_d2 >= 11'd64 && vp_d2 < 11'd80 || vp_d2 >= 11'd292 && vp_d2 < 11'd308 ||
                      vp_d2 >= 11'd520 && vp_d2 < 11'd536)) begin
            fix_en = 1'b1; fix_ch = "V"; fix_col = show_v ? CHIP_V_TXT : 24'h505860;
            fix_yo = vp_d2[3:0] - chip_toff;
        end else if (hp_d2 >= 12'd336 && hp_d2 < 12'd344 &&
                     (vp_d2 >= 11'd64 && vp_d2 < 11'd80 || vp_d2 >= 11'd292 && vp_d2 < 11'd308 ||
                      vp_d2 >= 11'd520 && vp_d2 < 11'd536)) begin
            fix_en = 1'b1; fix_ch = "I"; fix_col = show_i ? CHIP_I_TXT : 24'h505860;
            fix_yo = vp_d2[3:0] - chip_toff;
        end
        // 左缘电压刻度（x217..264，6 字符：±半窗/±1/3/0，自适应挡位+增益）
        else if (hp_d2 >= 12'd217 && hp_d2 < 12'd265 && show_v &&
                 (win_sel != 2'd3) && (scl_row != 3'd7)) begin
            fix_en = 1'b1; fix_col = 24'h98A4B8;
            fix_ch = scl_fmt(sd_v, scl_row, sl_off[5:3], 1'b0);
            fix_yo = wf_yoff[3:0];
        end
        // 右缘电流刻度（x1184..1231，同结构单位 A）
        else if (hp_d2 >= 12'd1184 && hp_d2 < 12'd1232 && show_i &&
                 (win_sel != 2'd3) && (scl_row != 3'd7)) begin
            fix_en = 1'b1; fix_col = 24'hCBB26A;
            fix_ch = scl_fmt(sd_i, scl_row, sr_off[5:3], 1'b1);
            fix_yo = wf_yoff[3:0];
        end
        // X 轴时间标签（默认时基 0..40ms）
        else if (vp_d2 >= 11'd704 && vp_d2 < 11'd720 && tb_px == 3'sd0) begin
            fix_en = 1'b1; fix_col = TXT_BODY; fix_yo = vp_d2[3:0];
            if      (hp_d2 >= 12'd216  && hp_d2 < 12'd224)  fix_ch = "0";
            else if (hp_d2 >= 12'd464  && hp_d2 < 12'd480)  fix_ch = (hp_d2 < 12'd472) ? "1" : "0";
            else if (hp_d2 >= 12'd720  && hp_d2 < 12'd736)  fix_ch = (hp_d2 < 12'd728) ? "2" : "0";
            else if (hp_d2 >= 12'd976  && hp_d2 < 12'd992)  fix_ch = (hp_d2 < 12'd984) ? "3" : "0";
            else if (hp_d2 >= 12'd1232 && hp_d2 < 12'd1248) fix_ch = (hp_d2 < 12'd1240) ? "4" : "0";
        end
    end else if (mode_px == 2'd1) begin
        // 频谱图例 + Y/X 轴（全部区域 16px 对齐，yo = vp[3:0]）
        if (hp_d2 >= 12'd240 && hp_d2 < 12'd248 && vp_d2 >= 11'd48 && vp_d2 < 11'd64) begin
            fix_en = 1'b1; fix_ch = "A"; fix_col = 24'h2A0D0D; fix_yo = vp_d2[3:0];
        end else if (hp_d2 >= 12'd264 && hp_d2 < 12'd272 && vp_d2 >= 11'd48 && vp_d2 < 11'd64) begin
            fix_en = 1'b1; fix_ch = "B"; fix_col = 24'h0A2A14; fix_yo = vp_d2[3:0];
        end else if (hp_d2 >= 12'd288 && hp_d2 < 12'd296 && vp_d2 >= 11'd48 && vp_d2 < 11'd64) begin
            fix_en = 1'b1; fix_ch = "C"; fix_col = 24'h0D1230; fix_yo = vp_d2[3:0];
        end else if (hp_d2 >= 12'd316 && hp_d2 < 12'd324 && vp_d2 >= 11'd48 && vp_d2 < 11'd64) begin
            fix_en = 1'b1; fix_ch = vi_cur ? "I" : "V";
            fix_col = vi_cur ? CHIP_I_TXT : CHIP_V_TXT;
            fix_yo = vp_d2[3:0];
        end else if (hp_d2 >= 12'd1184 && hp_d2 < 12'd1216 && sp_ytk_hit) begin
            fix_en = 1'b1; fix_col = TXT_BODY; fix_yo = vp_d2[3:0];
            if (vp_d2 >= 11'd656) begin
                fix_ch = (hp_d2[4:3] == 2'd0) ? "0" : (hp_d2[4:3] == 2'd1) ? "%" : " ";
            end else if (vp_d2 >= 11'd544) begin
                fix_ch = (hp_d2[4:3] == 2'd0) ? "2" : (hp_d2[4:3] == 2'd1) ? "0" :
                         (hp_d2[4:3] == 2'd2) ? "%" : " ";
            end else if (vp_d2 >= 11'd432) begin
                fix_ch = (hp_d2[4:3] == 2'd0) ? "4" : (hp_d2[4:3] == 2'd1) ? "0" :
                         (hp_d2[4:3] == 2'd2) ? "%" : " ";
            end else if (vp_d2 >= 11'd320) begin
                fix_ch = (hp_d2[4:3] == 2'd0) ? "6" : (hp_d2[4:3] == 2'd1) ? "0" :
                         (hp_d2[4:3] == 2'd2) ? "%" : " ";
            end else if (vp_d2 >= 11'd208) begin
                fix_ch = (hp_d2[4:3] == 2'd0) ? "8" : (hp_d2[4:3] == 2'd1) ? "0" :
                         (hp_d2[4:3] == 2'd2) ? "%" : " ";
            end else begin
                fix_ch = (hp_d2[4:3] == 2'd0) ? "1" : (hp_d2[4:3] == 2'd1) ? "0" :
                         (hp_d2[4:3] == 2'd2) ? "0" : "%";
            end
        end else if (vp_d2 >= 11'd672 && vp_d2 < 11'd688 &&
                     hp_d2 >= 12'd232 && hp_d2 < 12'd1176) begin
            fix_en = 1'b1; fix_col = TXT_BODY; fix_yo = vp_d2[3:0];
            if (xl_off[5:3] == 3'd3)
                fix_ch = (xl_off[9:6] < 4'd9) ? (8'h31 + {4'b0, xl_off[9:6]}) : (8'h27 + xl_off[9:6]);
            else if (xl_off[5:3] == 3'd2)
                fix_ch = (xl_off[9:6] >= 4'd9) ? "1" : " ";
            else fix_ch = " ";
        end else if (vp_d2 >= 11'd688 && vp_d2 < 11'd704 &&
                     hp_d2 >= 12'd640 && hp_d2 < 12'd752) begin
            fix_en = 1'b1; fix_col = TXT_DIM; fix_yo = vp_d2[3:0];
            case (hp_d2[10:3])
                7'd80: fix_ch = "H"; 7'd81: fix_ch = "A"; 7'd82: fix_ch = "R";
                7'd83: fix_ch = "M"; 7'd84: fix_ch = "O"; 7'd85: fix_ch = "N";
                7'd86: fix_ch = "I"; 7'd87: fix_ch = "C"; 7'd89: fix_ch = "O";
                7'd90: fix_ch = "R"; 7'd91: fix_ch = "D"; 7'd92: fix_ch = "E";
                7'd93: fix_ch = "R"; default: fix_ch = " ";
            endcase
        end
    end
end



// ---------------------------------------------------------------------------
// 8. 像素合成
// ---------------------------------------------------------------------------
reg [3:0] wf_layer;
always @* begin
    if      (chip_a || chip_b || chip_c ||
             vchip_a || vchip_b || vchip_c || ichip_a || ichip_b || ichip_c) wf_layer = 4'd0;
    else if (core_i)               wf_layer = 4'd1;
    else if (core_v)               wf_layer = 4'd2;
    else if (draw_v)               wf_layer = 4'd4;
    else if (mk_a || mk_b || mk_c) wf_layer = 4'd5;
    else if (wf_accent)            wf_layer = 4'd6;
    else if (wf_border)            wf_layer = 4'd7;
    else if (wf_mid)               wf_layer = 4'd8;
    else if (wf_vmain)             wf_layer = 4'd9;
    else if (wf_hgrid)             wf_layer = 4'd10;
    else if (wf_vgrid)             wf_layer = 4'd11;
    else if (wf_xtick)             wf_layer = 4'd12;
    else                           wf_layer = 4'd13;
end
wire vchip_ph_hidden = vchip_a ? !show_a_tr : vchip_b ? !show_b_tr : !show_c_tr;
wire ichip_ph_hidden = ichip_a ? !show_a_tr : ichip_b ? !show_b_tr : !show_c_tr;
wire [23:0] vchip_col = vchip_ph_hidden ? CHIP_DIM : (!show_v ? CHIP_V_DIM : CHIP_V);
wire [23:0] ichip_col = (ichip_ph_hidden || !show_i) ? CHIP_DIM : COL_I;
wire [23:0] chip_col = (ichip_a || ichip_b || ichip_c) ? ichip_col :
                       (vchip_a || vchip_b || vchip_c) ? vchip_col :
                       chip_a ? (!show_a_tr ? CHIP_DIM : COL_A) :
                       chip_b ? (!show_b_tr ? CHIP_DIM : COL_B) :
                                (!show_c_tr ? CHIP_DIM : COL_C);
wire [23:0] core_v_col = in_wfa ? COL_A : in_wfb ? COL_B : COL_C;
wire [23:0] glow_v_col = in_wfa ? GLOW_A : in_wfb ? GLOW_B : GLOW_C;
wire [23:0] mk_col = mk_a ? ACC_A : mk_b ? ACC_B : ACC_C;
wire [23:0] acc_col = in_wfa ? ACC_A : in_wfb ? ACC_B : ACC_C;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rgb_r <= 8'd0; rgb_g <= 8'd0; rgb_b <= 8'd0;
    end else if (!enable || !act_d3) begin
        rgb_r <= 8'd0; rgb_g <= 8'd0; rgb_b <= 8'd0;
    end else if (text_bit) begin
        rgb_r <= txt_col_s2[23:16];
        rgb_g <= txt_col_s2[15:8];
        rgb_b <= txt_col_s2[7:0];
    end else if (vy < 11'd32) begin
        if (run_fill) begin
            if (run_edge) begin
                rgb_r <= frz_s ? STOP_BORD[23:16] : RUN_BORD[23:16];
                rgb_g <= frz_s ? STOP_BORD[15:8]  : RUN_BORD[15:8];
                rgb_b <= frz_s ? STOP_BORD[7:0]   : RUN_BORD[7:0];
            end else begin
                rgb_r <= frz_s ? STOP_FILL[23:16] : RUN_FILL[23:16];
                rgb_g <= frz_s ? STOP_FILL[15:8]  : RUN_FILL[15:8];
                rgb_b <= frz_s ? STOP_FILL[7:0]   : RUN_FILL[7:0];
            end
        end else if (alm_fill) begin
            if (alm_act) begin
                if (blink_on) begin
                    rgb_r <= alm_edge ? ALM_BORD[23:16] : ALM_FILL[23:16];
                    rgb_g <= alm_edge ? ALM_BORD[15:8]  : ALM_FILL[15:8];
                    rgb_b <= alm_edge ? ALM_BORD[7:0]   : ALM_FILL[7:0];
                end else begin
                    rgb_r <= alm_edge ? ALM_DIMB[23:16] : ALM_DIMF[23:16];
                    rgb_g <= alm_edge ? ALM_DIMB[15:8]  : ALM_DIMF[15:8];
                    rgb_b <= alm_edge ? ALM_DIMB[7:0]   : ALM_DIMF[7:0];
                end
            end else begin
                rgb_r <= alm_edge ? ALM_OFFB[23:16] : ALM_OFFF[23:16];
                rgb_g <= alm_edge ? ALM_OFFB[15:8]  : ALM_OFFF[15:8];
                rgb_b <= alm_edge ? ALM_OFFB[7:0]   : ALM_OFFF[7:0];
            end
        end else if (pil_fill) begin
            rgb_r <= pil_edge ? mode_bord[23:16] : mode_fill[23:16];
            rgb_g <= pil_edge ? mode_bord[15:8]  : mode_fill[15:8];
            rgb_b <= pil_edge ? mode_bord[7:0]   : mode_fill[7:0];
        end else begin
            rgb_r <= BG_TOP[23:16]; rgb_g <= BG_TOP[15:8]; rgb_b <= BG_TOP[7:0];
        end
    end else if (vy == 11'd32) begin
        rgb_r <= ACC_LN[23:16]; rgb_g <= ACC_LN[15:8]; rgb_b <= ACC_LN[7:0];
    end else if (vy == 11'd33) begin
        rgb_r <= ACC_LN2[23:16]; rgb_g <= ACC_LN2[15:8]; rgb_b <= ACC_LN2[7:0];
    end else if (hx < 12'd211) begin
        if (hx >= 12'd208) begin
            rgb_r <= LINE_SEP[23:16]; rgb_g <= LINE_SEP[15:8]; rgb_b <= LINE_SEP[7:0];
        end else if ((vy == 11'd52) || (vy == 11'd680)) begin
            rgb_r <= LINE_DIV[23:16]; rgb_g <= LINE_DIV[15:8]; rgb_b <= LINE_DIV[7:0];
        end else if ((vy >= 11'd64) && (vy < 11'd544) && (hx <= 12'd2)) begin
            // 相色条与 15 带对齐：y64-159=A、y160-255=B、y256-351=C
            case (vy[9:5])
                5'd2, 5'd3, 5'd4: begin rgb_r <= ACC_A[23:16]; rgb_g <= ACC_A[15:8]; rgb_b <= ACC_A[7:0]; end
                5'd5, 5'd6, 5'd7: begin rgb_r <= ACC_B[23:16]; rgb_g <= ACC_B[15:8]; rgb_b <= ACC_B[7:0]; end
                5'd8, 5'd9, 5'd10: begin rgb_r <= ACC_C[23:16]; rgb_g <= ACC_C[15:8]; rgb_b <= ACC_C[7:0]; end
                default: begin rgb_r <= ACC_N[23:16]; rgb_g <= ACC_N[15:8]; rgb_b <= ACC_N[7:0]; end
            endcase
        end else begin
            rgb_r <= BG_PANEL[23:16]; rgb_g <= BG_PANEL[15:8]; rgb_b <= BG_PANEL[7:0];
        end
    end else if (mode_px == 2'd0) begin
        case (wf_layer)
            4'd0:    begin rgb_r <= chip_col[23:16];   rgb_g <= chip_col[15:8];   rgb_b <= chip_col[7:0];   end
            4'd1:    begin rgb_r <= COL_I[23:16];      rgb_g <= COL_I[15:8];      rgb_b <= COL_I[7:0];      end
            4'd2:    begin rgb_r <= core_v_col[23:16]; rgb_g <= core_v_col[15:8]; rgb_b <= core_v_col[7:0]; end
            4'd4:    begin rgb_r <= glow_v_col[23:16]; rgb_g <= glow_v_col[15:8]; rgb_b <= glow_v_col[7:0]; end
            4'd5:    begin rgb_r <= mk_col[23:16];     rgb_g <= mk_col[15:8];     rgb_b <= mk_col[7:0];     end
            4'd6:    begin rgb_r <= acc_col[23:16];     rgb_g <= acc_col[15:8];    rgb_b <= acc_col[7:0];    end
            4'd7:    begin rgb_r <= LINE_SEP[23:16];    rgb_g <= LINE_SEP[15:8];   rgb_b <= LINE_SEP[7:0];   end
            4'd8:    begin rgb_r <= GRID_MID[23:16];    rgb_g <= GRID_MID[15:8];   rgb_b <= GRID_MID[7:0];   end
            4'd9:    begin rgb_r <= GRID_VM[23:16];     rgb_g <= GRID_VM[15:8];    rgb_b <= GRID_VM[7:0];    end
            4'd10:   begin rgb_r <= GRID_H[23:16];      rgb_g <= GRID_H[15:8];     rgb_b <= GRID_H[7:0];     end
            4'd11:   begin rgb_r <= GRID_V[23:16];      rgb_g <= GRID_V[15:8];     rgb_b <= GRID_V[7:0];     end
            4'd12:   begin rgb_r <= AXIS[23:16];        rgb_g <= AXIS[15:8];       rgb_b <= AXIS[7:0];       end
            default: begin rgb_r <= BG_MAIN[23:16];     rgb_g <= BG_MAIN[15:8];    rgb_b <= BG_MAIN[7:0];    end
        endcase
    end else if (mode_px == 2'd1) begin
        if (lc_a) begin
            rgb_r <= COL_A[23:16]; rgb_g <= COL_A[15:8]; rgb_b <= COL_A[7:0];
        end else if (lc_b) begin
            rgb_r <= COL_B[23:16]; rgb_g <= COL_B[15:8]; rgb_b <= COL_B[7:0];
        end else if (lc_c) begin
            rgb_r <= COL_C[23:16]; rgb_g <= COL_C[15:8]; rgb_b <= COL_C[7:0];
        end else if (lc_i) begin
            rgb_r <= COL_I[23:16]; rgb_g <= COL_I[15:8]; rgb_b <= COL_I[7:0];
        end else if (draw_sp) begin
            case (sp_bar_g_q2)
                2'd0: begin
                    rgb_r <= sp_cap ? CAP_A[23:16] : COL_A[23:16];
                    rgb_g <= sp_cap ? CAP_A[15:8]  : COL_A[15:8];
                    rgb_b <= sp_cap ? CAP_A[7:0]   : COL_A[7:0];
                end
                2'd1: begin
                    rgb_r <= sp_cap ? CAP_B[23:16] : COL_B[23:16];
                    rgb_g <= sp_cap ? CAP_B[15:8]  : COL_B[15:8];
                    rgb_b <= sp_cap ? CAP_B[7:0]   : COL_B[7:0];
                end
                default: begin
                    rgb_r <= sp_cap ? CAP_C[23:16] : COL_C[23:16];
                    rgb_g <= sp_cap ? CAP_C[15:8]  : COL_C[15:8];
                    rgb_b <= sp_cap ? CAP_C[7:0]   : COL_C[7:0];
                end
            endcase
        end else if (sp_base || sp_yaxis || sp_ytick) begin
            rgb_r <= AXIS[23:16]; rgb_g <= AXIS[15:8]; rgb_b <= AXIS[7:0];
        end else if (sp_hgrid) begin
            rgb_r <= GRID_H[23:16]; rgb_g <= GRID_H[15:8]; rgb_b <= GRID_H[7:0];
        end else if (sp_grid) begin
            rgb_r <= GRID_V[23:16]; rgb_g <= GRID_V[15:8]; rgb_b <= GRID_V[7:0];
        end else begin
            rgb_r <= BG_MAIN[23:16]; rgb_g <= BG_MAIN[15:8]; rgb_b <= BG_MAIN[7:0];
        end
    end else if (mode_px == 2'd2) begin
        if (tbl_rect) begin
            if (tbl_border) begin
                rgb_r <= LINE_SEP[23:16]; rgb_g <= LINE_SEP[15:8]; rgb_b <= LINE_SEP[7:0];
            end else if (tbl_hdrbg) begin
                rgb_r <= TBL_HDRBG[23:16]; rgb_g <= TBL_HDRBG[15:8]; rgb_b <= TBL_HDRBG[7:0];
            end else if (tbl_hline) begin
                rgb_r <= TBL_LINE[23:16]; rgb_g <= TBL_LINE[15:8]; rgb_b <= TBL_LINE[7:0];
            end else if (tbl_drow && tbl_tint_a) begin
                rgb_r <= TBL_ABG[23:16]; rgb_g <= TBL_ABG[15:8]; rgb_b <= TBL_ABG[7:0];
            end else if (tbl_drow && tbl_tint_b) begin
                rgb_r <= TBL_BBG[23:16]; rgb_g <= TBL_BBG[15:8]; rgb_b <= TBL_BBG[7:0];
            end else if (tbl_drow && tbl_tint_c) begin
                rgb_r <= TBL_CBG[23:16]; rgb_g <= TBL_CBG[15:8]; rgb_b <= TBL_CBG[7:0];
            end else begin
                rgb_r <= BG_TABLE[23:16]; rgb_g <= BG_TABLE[15:8]; rgb_b <= BG_TABLE[7:0];
            end
        end else begin
            rgb_r <= BG_MAIN[23:16]; rgb_g <= BG_MAIN[15:8]; rgb_b <= BG_MAIN[7:0];
        end
    end
end

endmodule
