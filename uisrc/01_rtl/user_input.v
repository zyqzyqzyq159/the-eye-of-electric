// 按键板输入控制：6 按键 + 4 旋转编码器 + 状态机（外设按键板，键帽下 WS2812）
//
// 按键（上排 3 个全局键 + 下排 3 个情境键，全部低有效、外接上拉；核心板 SW2/SW3 高有效等效备份）：
//   BTN_FUNC   功能模式循环：0=自检 1=单相A 2=三相四线 3=三相三线（上电默认自检）
//   BTN_VIEW   显示模式循环：0=示波 1=频谱 2=谐波表
//   BTN_FREEZE 定格 RUN/STOP（按一次定格，再按恢复）
//   自检模式下：SEL1=谐波注入 SEL2=不平衡注入 SEL3=逆相序演示开关
//   三相测量模式下：SEL1=通道循环(三相全显→A→B→C)
//                  SEL2=V/I 三态循环(双显V+I→仅电压→仅电流)
//                  SEL3=选中波形目标(全→A→B→C)
// 编码器（增量正交，A/B 两个开关通道 + 公共端 GND，输出无方向极性约定，反了交换 A/B 接线即可）：
//   ENC1 时基（-1..2 → 窗口 1/2/4/8 周期）  ENC2 平移（每格 ±8 样本）
//   ENC3 垂直位置（每格 ±8px，作用于选中窗）
//   ENC4 垂直量程（在自适应挡位上 -4..+3 挡偏移，V/A 同调，作用于选中窗）：
//         顺时针 = 量程变粗（波形变小），逆时针 = 量程变细（波形变大）。
//         固定 1x/2x/4x 增益在自适应挡位（恒把波形放到 ~62% 半窗）下必然削顶，
//         2x 与 4x 视觉无差异——故改为示波器式 V/DIV 挡位偏移，每格都有可见缩放
//   ENC1/3/4 带按压复位：ENC1_SW=时基回默认 2 周期，ENC3_SW=三窗垂直位置回 0，
//   ENC4_SW=三窗量程偏移清零（恢复自适应挡位；复位为全局作用，不随选中目标）
//
// 输出两组 CDC 快照（数据先更新、2 拍后翻转握手；display_ctrl 在翻转沿整体锁存）：
//   func_data/func_toggle  功能状态快照 {func_mode[1:0], meas_chan[1:0], meas_vi[1:0],
//                                        self_sel[2:0], meas_pick[1:0]}（11bit）
//   view_data/view_toggle  视图参数快照 {tb s3, pan9, vpos×3 s8, soff×3 s3}（45bit）
module user_input #(
    parameter CLK_FREQ = 100_000_000,
    parameter DEBOUNCE_MS = 1,
    parameter EXT_BTN_ACTIVE_LOW = 1   // 按键板键帽接地+上拉=低有效；核心板 SW2/SW3 按下为高
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        btn_func,
    input  wire        btn_view,
    input  wire        btn_freeze,
    input  wire        btn_sel1,
    input  wire        btn_sel2,
    input  wire        btn_sel3,
    input  wire        btn_sw2,          // 核心板备份键 = VIEW
    input  wire        btn_sw3,          // 核心板备份键 = FREEZE

    input  wire        enc1_a, enc1_b,   // 时基
    input  wire        enc1_sw,          // 时基按压复位（tb 回 0 = 2 周期默认窗）
    input  wire        enc2_a, enc2_b,   // 平移
    input  wire        enc3_a, enc3_b,   // 垂直位置
    input  wire        enc3_sw,          // 垂直位置按压复位（三窗 vpos 回 0）
    input  wire        enc4_a, enc4_b,   // 垂直缩放
    input  wire        enc4_sw,          // 垂直量程按压复位（三窗偏移清零=回自适应挡位）

    output reg  [1:0]  func_mode,
    output reg  [1:0]  disp_mode,
    output reg         freeze,
    output reg  [2:0]  self_sel,
    output reg  [1:0]  meas_chan,
    output reg  [1:0]  meas_vi,       // 0=双显(V+I 叠加) 1=仅电压 2=仅电流
    output reg  [1:0]  meas_pick,
    output reg  [10:0] func_data,
    output reg         func_toggle,
    output reg  [44:0] view_data,
    output reg         view_toggle,
    output reg         mode_change_pulse,

    output wire [9:0]  ui_event        // 单拍脉冲供 WS2812 灯效：[5:0]=6 键，
                                        // [6..9]=编码器 1..4 步进或按压复位
);

localparam DEBOUNCE_COUNT = CLK_FREQ / 1000 * DEBOUNCE_MS;

// ---------------------------------------------------------------------------
// 1. 按键消抖（11 路同一条流水线：6 板外键 + 3 编码器按压 + SW2/SW3）
// ---------------------------------------------------------------------------
wire [8:0] ext_btn = EXT_BTN_ACTIVE_LOW ? ~{enc4_sw, enc3_sw, enc1_sw,
                                            btn_sel3, btn_sel2, btn_sel1, btn_freeze, btn_view, btn_func}
                                        : {enc4_sw, enc3_sw, enc1_sw,
                                           btn_sel3, btn_sel2, btn_sel1, btn_freeze, btn_view, btn_func};
wire [10:0] btn_raw = {ext_btn, btn_sw3, btn_sw2};  // bit0=SW2 bit1=SW3 bit2=FUNC bit3=VIEW
                                                    // bit4=FREEZE bit5=SEL1 bit6=SEL2 bit7=SEL3
                                                    // bit8=ENC1_SW bit9=ENC3_SW bit10=ENC4_SW
reg [1:0]  btn_sync [0:10];
reg [19:0] btn_cnt  [0:10];
reg [10:0] btn_deb, btn_prev;
integer bi;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (bi = 0; bi < 11; bi = bi + 1) begin
            btn_sync[bi] <= 2'b00;
            btn_cnt[bi]  <= 20'd0;
        end
        btn_deb <= 11'd0;
    end else begin
        for (bi = 0; bi < 11; bi = bi + 1) begin
            btn_sync[bi] <= {btn_sync[bi][0], btn_raw[bi]};
            if (btn_sync[bi][1] == btn_deb[bi]) begin
                btn_cnt[bi] <= 20'd0;
            end else begin
                btn_cnt[bi] <= btn_cnt[bi] + 1'b1;
                if (btn_cnt[bi] >= DEBOUNCE_COUNT[19:0]) begin
                    btn_deb[bi] <= btn_sync[bi][1];
                    btn_cnt[bi] <= 20'd0;
                end
            end
        end
    end
end

always @(posedge clk) btn_prev <= btn_deb;

wire [10:0] btn_press = btn_deb & ~btn_prev;
wire p_sw2   = btn_press[0];
wire p_sw3   = btn_press[1];
wire p_func  = btn_press[2];
wire p_view  = btn_press[3] | p_sw2;
wire p_frz   = btn_press[4] | p_sw3;
wire p_s1    = btn_press[5];
wire p_s2    = btn_press[6];
wire p_s3    = btn_press[7];
wire p_e1rst = btn_press[8];    // ENC1 按压：时基复位
wire p_e3rst = btn_press[9];    // ENC3 按压：垂直位置复位
wire p_e4rst = btn_press[10];   // ENC4 按压：垂直缩放复位

// ---------------------------------------------------------------------------
// 2. 编码器正交解码（四分之一计数制）
//    格雷序列 00→01→11→10→00 为 +，反序为 −；EC11 每格产生一个完整正交周期
//    （4 个格雷跳变），累计满 4 个同向跳变输出 1 步——单格单步且天然滤除
//    触点抖动（±1 跳变交替抵消，不产生假步进）。非法跳变（两线同变）丢弃。
// ---------------------------------------------------------------------------
wire [7:0] enc_raw = {enc4_b, enc4_a, enc3_b, enc3_a, enc2_b, enc2_a, enc1_b, enc1_a};

reg [1:0]  enc_sa [0:3];
reg [1:0]  enc_sb [0:3];
reg [1:0]  enc_p  [0:3];    // {A,B} 前拍状态
reg signed [2:0] enc_acc[0:3];  // 四分之一格累加 -3..+3
reg [1:0]  enc_st [0:3];    // 本拍输出 0=无 1=+1 步 2=-1 步
integer ei;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (ei = 0; ei < 4; ei = ei + 1) begin
            enc_sa[ei] <= 2'b00; enc_sb[ei] <= 2'b00;
            enc_p[ei]  <= 2'b00;
            enc_acc[ei] <= 3'sd0;
            enc_st[ei] <= 2'd0;
        end
    end else begin
        for (ei = 0; ei < 4; ei = ei + 1) begin
            enc_sa[ei] <= {enc_sa[ei][0], enc_raw[ei*2]};
            enc_sb[ei] <= {enc_sb[ei][0], enc_raw[ei*2+1]};
            enc_st[ei] <= 2'd0;
            if ({enc_sa[ei][1], enc_sb[ei][1]} != enc_p[ei]) begin
                case ({enc_p[ei], enc_sa[ei][1], enc_sb[ei][1]})
                    4'b0010, 4'b1011, 4'b1101, 4'b0100: begin
                        if (enc_acc[ei] == 3'sd3) begin
                            enc_acc[ei] <= 3'sd0;
                            enc_st[ei]  <= 2'd1;
                        end else begin
                            enc_acc[ei] <= enc_acc[ei] + 3'sd1;
                        end
                    end
                    4'b0001, 4'b0111, 4'b1110, 4'b1000: begin
                        if (enc_acc[ei] == -3'sd3) begin
                            enc_acc[ei] <= 3'sd0;
                            enc_st[ei]  <= 2'd2;
                        end else begin
                            enc_acc[ei] <= enc_acc[ei] - 3'sd1;
                        end
                    end
                    default: ;          // 非法跳变（两线同变）丢弃
                endcase
                enc_p[ei] <= {enc_sa[ei][1], enc_sb[ei][1]};
            end
        end
    end
end

// 灯效事件：6 个板外键按压 + 4 个编码器（步进或按压复位）各一拍
assign ui_event = {(enc_st[3] != 2'd0) | p_e4rst,
                   (enc_st[2] != 2'd0) | p_e3rst,
                   (enc_st[1] != 2'd0),
                   (enc_st[0] != 2'd0) | p_e1rst,
                   btn_press[7:2]};

// ---------------------------------------------------------------------------
// 3. 功能状态机（按键）
// ---------------------------------------------------------------------------
reg func_req;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        func_mode <= 2'd0;         // 上电默认自检（未接采样板即可演示）
        disp_mode <= 2'd0;
        freeze    <= 1'b0;
        self_sel  <= 3'd0;
        meas_chan <= 2'd0;
        meas_vi   <= 2'd0;         // 上电默认双显（V 相色 + I 琥珀叠加）
        meas_pick <= 2'd0;
        mode_change_pulse <= 1'b0;
        func_req  <= 1'b0;
    end else begin
        mode_change_pulse <= 1'b0;
        func_req <= 1'b0;

        if (p_func) begin
            func_mode <= (func_mode == 2'd3) ? 2'd0 : func_mode + 2'd1;
            func_req <= 1'b1;
        end

        if (p_view) begin
            disp_mode <= (disp_mode == 2'd2) ? 2'd0 : disp_mode + 2'd1;  // 0示波/1频谱/2谐波表（3 态循环）
            mode_change_pulse <= 1'b1;
        end

        if (p_frz) freeze <= ~freeze;

        if (func_mode == 2'd0) begin
            // 自检模式：SEL1/2/3 = 谐波/不平衡/逆相序注入开关
            if (p_s1) begin self_sel[0] <= ~self_sel[0]; func_req <= 1'b1; end
            if (p_s2) begin self_sel[1] <= ~self_sel[1]; func_req <= 1'b1; end
            if (p_s3) begin self_sel[2] <= ~self_sel[2]; func_req <= 1'b1; end
        end else begin
            // 测量模式：SEL1=通道循环 SEL2=V/I 三态 SEL3=选中波形目标
            if (p_s1) begin meas_chan <= (meas_chan == 2'd3) ? 2'd0 : meas_chan + 2'd1; func_req <= 1'b1; end
            if (p_s2) begin meas_vi <= (meas_vi == 2'd2) ? 2'd0 : meas_vi + 2'd1; func_req <= 1'b1; end
            if (p_s3) begin meas_pick <= (meas_pick == 2'd3) ? 2'd0 : meas_pick + 2'd1; func_req <= 1'b1; end
        end
    end
end

// ---------------------------------------------------------------------------
// 4. 视图参数（编码器累加）
// ---------------------------------------------------------------------------
reg signed [2:0] vw_tb;     // -1..2：窗口 1/2/4/8 周期
reg [8:0]  vw_pan;          // 0..511 回绕（±8 样本/格）
reg signed [7:0] vw_vpos [0:2];   // ±60px 限幅
reg signed [2:0] vw_soff [0:2];   // 量程偏移 -4..+3（叠加在自适应挡位上，V/A 同调）
reg        vw_req;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vw_tb <= 3'sd0;
        vw_pan <= 9'd0;
        for (ei = 0; ei < 3; ei = ei + 1) begin
            vw_vpos[ei] <= 8'sd0;
            vw_soff[ei] <= 3'sd0;
        end
        vw_req <= 1'b0;
    end else begin
        vw_req <= 1'b0;

        // ENC1 时基：-1..2 限幅；按压复位回默认 2 周期
        if (p_e1rst) begin
            vw_tb <= 3'sd0; vw_req <= 1'b1;
        end else if (enc_st[0] == 2'd1 && vw_tb < 3'sd2) begin
            vw_tb <= vw_tb + 3'sd1; vw_req <= 1'b1;
        end else if (enc_st[0] == 2'd2 && vw_tb > -3'sd1) begin
            vw_tb <= vw_tb - 3'sd1; vw_req <= 1'b1;
        end

        // ENC2 平移：±8 样本/格，9bit 回绕
        if (enc_st[1] == 2'd1) begin
            vw_pan <= vw_pan + 9'd8; vw_req <= 1'b1;
        end else if (enc_st[1] == 2'd2) begin
            vw_pan <= vw_pan - 9'd8; vw_req <= 1'b1;
        end

        // ENC3 垂直位置：±8px/格，±60 限幅，作用于选中窗；按压复位三窗全回 0
        if (p_e3rst) begin
            for (ei = 0; ei < 3; ei = ei + 1) vw_vpos[ei] <= 8'sd0;
            vw_req <= 1'b1;
        end else if (enc_st[2] != 2'd0) begin
            for (ei = 0; ei < 3; ei = ei + 1) begin
                if (meas_pick == 2'd0 || meas_pick == (ei + 1)) begin
                    if (enc_st[2] == 2'd1 && vw_vpos[ei] < 8'sd52)
                        vw_vpos[ei] <= vw_vpos[ei] + 8'sd8;
                    else if (enc_st[2] == 2'd2 && vw_vpos[ei] > -8'sd52)
                        vw_vpos[ei] <= vw_vpos[ei] - 8'sd8;
                end
            end
            vw_req <= 1'b1;
        end

        // ENC4 垂直量程：自适应挡位上偏移 -4..+3（CW=粗挡/波形变小），
        // 作用于选中窗；按压复位三窗偏移清零（回自适应挡位）
        if (p_e4rst) begin
            for (ei = 0; ei < 3; ei = ei + 1) vw_soff[ei] <= 3'sd0;
            vw_req <= 1'b1;
        end else if (enc_st[3] != 2'd0) begin
            for (ei = 0; ei < 3; ei = ei + 1) begin
                if (meas_pick == 2'd0 || meas_pick == (ei + 1)) begin
                    if (enc_st[3] == 2'd1 && vw_soff[ei] < 3'sd3)
                        vw_soff[ei] <= vw_soff[ei] + 3'sd1;
                    else if (enc_st[3] == 2'd2 && vw_soff[ei] > -3'sd4)
                        vw_soff[ei] <= vw_soff[ei] - 3'sd1;
                end
            end
            vw_req <= 1'b1;
        end
    end
end

// ---------------------------------------------------------------------------
// 5. CDC 快照与翻转握手（数据稳定 2 拍后翻转，接收域翻转沿锁存）
// ---------------------------------------------------------------------------
reg [1:0] ft_dly;
reg [1:0] vt_dly;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        func_data <= 11'd0; func_toggle <= 1'b0; ft_dly <= 2'd0;
    end else begin
        if (func_req) ft_dly <= 2'd2;
        else if (ft_dly != 2'd0) begin
            ft_dly <= ft_dly - 1'b1;
            if (ft_dly == 2'd1) begin
                func_toggle <= ~func_toggle;
                func_data   <= {func_mode, meas_chan, meas_vi, self_sel, meas_pick};
            end
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        view_data <= 45'd0; view_toggle <= 1'b0; vt_dly <= 2'd0;
    end else begin
        if (vw_req) vt_dly <= 2'd2;
        else if (vt_dly != 2'd0) begin
            vt_dly <= vt_dly - 1'b1;
            if (vt_dly == 2'd1) begin
                view_toggle <= ~view_toggle;
                view_data   <= {vw_tb, vw_pan, vw_vpos[0], vw_vpos[1], vw_vpos[2],
                                vw_soff[0], vw_soff[1], vw_soff[2]};
            end
        end
    end
end

endmodule
