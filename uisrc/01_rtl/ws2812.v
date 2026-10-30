// WS2812 RGB 灯链驱动：按键板 10 颗灯（6 键帽 + 4 编码器各 1 颗），状态联动灯效
//
// 灯位（NLED=10，DIN 从 LED0 起串行，PCB 按面板左→右布局走线）：
//   0 BTN_FUNC  1 BTN_VIEW  2 BTN_FREEZE  3 BTN_SEL1  4 BTN_SEL2  5 BTN_SEL3
//   6 ENC1 时基  7 ENC2 平移  8 ENC3 垂直位置  9 ENC4 垂直缩放
//
// 灯效（帧率 ≈ 1.2kHz，每帧 300µs 数据 + 500µs 复位/计算窗口）：
//   ① 开机彩虹波：上电 ~1s 三通道相位错开的彩虹从面板扫过，随后直接进常态
//   ② 状态色（常驻基色）：FUNC=功能模式色（自检青/单相绿/四线琥珀/三线紫），
//      VIEW=显示模式色（青/琥珀/紫），FREEZE=运行绿/定格红，
//      SEL1-3=自检注入洋红/通道相色/V-I/选中色，编码器=暗青
//   ③ 呼吸波：全灯慢呼吸（~0.85s 周期），相位按灯位错开 → 一道微光沿面板流动
//   ④ 操作亮脉冲：被操作的键/旋钮灯 ~0.2s 亮脉冲（瞬时点亮、线性衰减）
//   ⑤ 报警：事件报警时整排红色脉动（~0.43s 周期，与屏幕 ALARM 闪烁同节奏）
//   ⑥ 定格时 FREEZE 灯常亮（退出呼吸）
//
// 亮度实现（无乘法器，保证 100MHz 时序）：
//   目标亮度 B(6bit, 32=1x) → 5 档移位缩放（0.5/0.75/1/1.25/1.5x，
//   每档 = 一次移位 + 一次加法），档内余数用 8 帧时域抖动（150Hz，不可察觉）
//   细分 → 有效亮度 32 级，呼吸平滑。曾用 8x6 乘法（DSP 或 6 级加法链）均超时序。
//
// 位时序（100MHz 拍数，符合 WS2812B 手册 ±150ns 容差）：
//   位周期 1.25µs；0 码 0.4µs 高；1 码 0.8µs 高。
//   （更早版本 1 码高电平误用位周期的 4/5 = 1.0µs，超出规格，实机会误码）
module ws2812 #(
    parameter NLED   = 10,
    parameter CLK_HZ = 100_000_000
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [1:0]  func_mode,
    input  wire [1:0]  disp_mode,
    input  wire        freeze,
    input  wire [2:0]  self_sel,
    input  wire [1:0]  meas_chan,
    input  wire [1:0]  meas_vi,      // 0=双显 1=仅电压 2=仅电流
    input  wire [1:0]  meas_pick,
    input  wire [9:0]  ui_event,      // 100MHz 同域单拍脉冲：[5:0]=6 键 [9:6]=编码器1..4
    input  wire [2:0]  event_alarm,

    output reg         din
);

// ---------------------------------------------------------------------------
// 1. 时序参数
// ---------------------------------------------------------------------------
localparam BIT_T  = CLK_HZ / 800_000;        // 125 拍 = 1.25µs 位周期
localparam ONE_H  = BIT_T * 16 / 25;         // 80 拍 = 0.8µs
localparam ZERO_H = BIT_T * 8 / 25;          // 40 拍 = 0.4µs
localparam RST_T  = CLK_HZ / 2000;           // 500µs 复位锁存

localparam SWEEP_FRAMES = 1200;              // 开机彩虹 ~1s

// ---------------------------------------------------------------------------
// 2. 输入同步（状态准静态 2 拍；事件同域直接锁存进 sticky）
// ---------------------------------------------------------------------------
reg [13:0] st_m, st_s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin st_m <= 14'd0; st_s <= 14'd0; end
    else begin
        st_m <= {func_mode, disp_mode, freeze, self_sel, meas_chan, meas_vi, meas_pick};
        st_s <= st_m;
    end
end
wire [1:0] s_func = st_s[13:12];
wire [1:0] s_disp = st_s[11:10];
wire       s_frz  = st_s[9];
wire [2:0] s_self = st_s[8:6];
wire [1:0] s_chan = st_s[5:4];
wire [1:0] s_vi   = st_s[3:2];
wire [1:0] s_pick = st_s[1:0];

reg [2:0] alm_m, alm_s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin alm_m <= 3'd0; alm_s <= 3'd0; end
    else begin alm_m <= event_alarm; alm_s <= alm_m; end
end
wire alm_act = (alm_s != 3'd0);

// ---------------------------------------------------------------------------
// 3. 状态基色（按灯号的组合函数）
// ---------------------------------------------------------------------------
localparam [23:0] C_FUNC0 = 24'h00_32_3C;   // 自检 青
localparam [23:0] C_FUNC1 = 24'h00_32_14;   // 单相 绿
localparam [23:0] C_FUNC2 = 24'h32_20_00;   // 三相四线 琥珀
localparam [23:0] C_FUNC3 = 24'h28_0E_32;   // 三相三线 紫
localparam [23:0] C_DISP0 = 24'h00_32_3C;
localparam [23:0] C_DISP1 = 24'h32_20_00;
localparam [23:0] C_DISP2 = 24'h28_0E_32;
localparam [23:0] C_RUN   = 24'h0C_3C_12;
localparam [23:0] C_STOP  = 24'h46_0A_0A;
localparam [23:0] C_ONM   = 24'h32_08_32;   // 注入开启 亮洋红
localparam [23:0] C_DIM   = 24'h08_08_0A;   // 关闭 微亮灰
localparam [23:0] C_CHALL = 24'h1A_1A_1C;   // 通道循环：全显
localparam [23:0] C_CHA   = 24'h3C_06_06;   // A
localparam [23:0] C_CHB   = 24'h06_36_0C;   // B
localparam [23:0] C_CHC   = 24'h06_12_3C;   // C
localparam [23:0] C_V     = 24'h08_1C_28;   // 电压
localparam [23:0] C_I     = 24'h3A_28_06;   // 电流
localparam [23:0] C_VI    = 24'h0A_2C_32;   // 双显（蓝绿混合）
localparam [23:0] C_ONW   = 24'h28_28_2C;   // 选中激活
localparam [23:0] C_ENC   = 24'h02_0E_14;   // 编码器 暗青

function [23:0] base_col;
    input [3:0] i;                           // 灯号 0..9
    begin
        case (i)
            4'd0: case (s_func) 2'd0: base_col = C_FUNC0; 2'd1: base_col = C_FUNC1;
                         2'd2: base_col = C_FUNC2; default: base_col = C_FUNC3; endcase
            4'd1: case (s_disp) 2'd0: base_col = C_DISP0; 2'd1: base_col = C_DISP1;
                         default: base_col = C_DISP2; endcase
            4'd2:    base_col = s_frz ? C_STOP : C_RUN;
            4'd3:    base_col = (s_func == 2'd0) ? (s_self[0] ? C_ONM : C_DIM) :
                                (s_chan == 2'd0) ? C_CHALL :
                                (s_chan == 2'd1) ? C_CHA   :
                                (s_chan == 2'd2) ? C_CHB   : C_CHC;
            4'd4:    base_col = (s_func == 2'd0) ? (s_self[1] ? C_ONM : C_DIM) :
                                (s_vi == 2'd1) ? C_V :
                                (s_vi == 2'd2) ? C_I : C_VI;
            4'd5:    base_col = (s_func == 2'd0) ? (s_self[2] ? C_ONM : C_DIM) :
                                ((s_pick != 2'd0) ? C_ONW : C_DIM);
            default: base_col = C_ENC;
        endcase
    end
endfunction

// ---------------------------------------------------------------------------
// 4. 动画寄存器与亮度计算（加法/比较，无乘法）
// ---------------------------------------------------------------------------
// 10bit 锯齿 → 0..255 三角波（周期 1024 帧）
function [7:0] tri10;
    input [9:0] t;
    reg [9:0] tr;
    begin
        tr = t[9] ? (10'd1023 - t) : t;
        tri10 = tr[8:1];
    end
endfunction

reg [10:0] frm_cnt;                          // 自由跑：呼吸/报警/抖动相位源
reg [10:0] boot_cnt;                         // 开机阶段计数（饱和）
reg [7:0]  flash   [0:NLED-1];               // 操作亮脉冲，255 起每帧 -1（~0.2s 衰减）
reg [9:0]  evt_stk;                          // 事件粘滞，帧尾消费

wire boot_sweep_on = (boot_cnt < SWEEP_FRAMES);

// B → 缩放档位：band = clamp((B-32)>>>3 + 抖动进位, -3, 2)
//   档位：-3=熄灭 -2=0.5x -1=0.75x 0=1x +1=1.25x +2=1.5x；档内 8 帧抖动细分
function signed [3:0] band_of;
    input [5:0] b;
    input [2:0] dith_ph;
    reg signed [6:0] t;
    reg signed [3:0] q;
    begin
        t = $signed({1'b0, b}) - 7'sd32;
        q = t >>> 3;
        if (dith_ph < b[2:0]) q = q + 4'sd1;   // 档内时域抖动（150Hz，不可察觉）
        if (q < -4'sd3)      band_of = -4'sd3;
        else if (q > 4'sd2)  band_of = 4'sd2;
        else                 band_of = q;
    end
endfunction

// 单通道移位缩放（一次移位 + 一次加法，无乘法）
function [7:0] scale_ch;
    input [7:0] c;
    input signed [3:0] q;
    begin
        case (q)
            -4'sd3:  scale_ch = 8'd0;
            -4'sd2:  scale_ch = {1'b0, c[7:1]};
            -4'sd1:  scale_ch = c - {2'b00, c[7:2]};
            4'sd1:   scale_ch = c + {2'b00, c[7:2]};
            4'sd2:   scale_ch = c + {1'b0, c[7:1]};
            default: scale_ch = c;
        endcase
    end
endfunction

wire frame_end;                               // 发送最后一拍（FSM 驱动）

// 事件粘滞：帧间持续捕获（否则帧中间到达的操作事件会丢），帧尾消费并保留同拍新事件
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) evt_stk <= 10'd0;
    else if (frame_end) evt_stk <= ui_event;
    else evt_stk <= evt_stk | ui_event;
end

// ---------------------------------------------------------------------------
// 5. 帧状态机：发送 240bit → 计算相位（10 拍亮度 + 10 灯×4 拍流水）→ 复位等待 → 装载
// ---------------------------------------------------------------------------
reg [23:0] col_reg [0:NLED-1];               // 本帧各灯 RGB（发送数据源）
reg [24*NLED-1:0] shreg;
reg        bit_lvl;
reg [7:0]  bit_cnt;                          // 24*NLED = 240 ≤ 255
reg [6:0]  tick;
reg [15:0] rst_cnt;
reg [5:0]  calc_k;                           // 0..49：10 灯 × 5 拍流水
reg        sending, calc_ph;
reg [2:0]  c_st;                              // 流水级 0..4
reg [3:0]  c_led2;                            // 流水当前灯号
reg [23:0] bc_reg;                            // 级0：基色
reg [7:0]  bf_reg;                            // 级0：呼吸三角波值
reg [7:0]  fl_reg;                            // 级0：操作脉冲值
reg [3:0]  wr_ad;                             // 级0：写地址
reg [5:0]  mv_reg;                            // 级1：亮度 B
reg signed [3:0] q_reg;                       // 级2：缩放档位
reg [23:0] rgb_word;                          // 级3：缩放后的整灯 RGB

wire [6:0] lvl_h = bit_lvl ? ONE_H[6:0] : ZERO_H[6:0];
wire frame_load = !sending && !calc_ph && (rst_cnt >= RST_T[15:0]);   // 帧装载拍
assign frame_end = sending && (tick >= BIT_T[6:0] - 7'd1) && (bit_cnt == 8'd0);

// 开机彩虹相位（wr_ad×64 纯拼接，不用乘法；64/1024 周期错开 10 灯成行波）
wire [9:0] sw_bt = {boot_cnt[8:0], 1'b0} + {4'd0, wr_ad, 6'd0};

integer ai;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        din <= 1'b0; sending <= 1'b0; calc_ph <= 1'b0;
        rst_cnt <= 16'd0; calc_k <= 6'd0; c_st <= 3'd0; c_led2 <= 4'd0;
        bit_cnt <= 8'd0; tick <= 7'd0;
        shreg <= {(24*NLED){1'b0}}; bit_lvl <= 1'b0;
        frm_cnt <= 11'd0; boot_cnt <= 11'd0;
        rgb_word <= 24'd0; wr_ad <= 4'd0; q_reg <= 4'sd0;
        bc_reg <= 24'd0; mv_reg <= 6'd0; bf_reg <= 8'd0; fl_reg <= 8'd0;
        for (ai = 0; ai < NLED; ai = ai + 1) begin
            flash[ai] <= 8'd0; col_reg[ai] <= 24'd0;
        end
    end else if (sending) begin
        // ---- 移位发送 ----
        din <= (tick < lvl_h);
        if (tick >= BIT_T[6:0] - 7'd1) begin
            tick    <= 7'd0;
            shreg   <= {shreg[24*NLED-2:0], 1'b0};
            bit_lvl <= shreg[24*NLED-2];
            if (bit_cnt == 8'd0) begin
                // 帧结束：动画寄存器前进一步，进入计算相位
                sending <= 1'b0;
                calc_ph <= 1'b1;
                calc_k  <= 6'd0;
                c_st    <= 3'd0;
                c_led2  <= 4'd0;
                rst_cnt <= 16'd0;
                frm_cnt <= frm_cnt + 11'd1;
                if (boot_cnt < SWEEP_FRAMES) boot_cnt <= boot_cnt + 11'd1;
                for (ai = 0; ai < NLED; ai = ai + 1)
                    flash[ai] <= evt_stk[ai] ? 8'd255 :
                                 (flash[ai] != 8'd0 ? flash[ai] - 8'd1 : 8'd0);
            end else begin
                bit_cnt <= bit_cnt - 8'd1;
            end
        end else begin
            tick <= tick + 7'd1;
        end
    end else if (calc_ph) begin
        // ---- 复位窗口内的时分复用计算：din 保持 0 ----
        din <= 1'b0;
        begin
            // 每灯 5 拍流水：三角波/脉冲取值 → 亮度 B → 档位 → 移位缩放 → 写 RAM
            // （每级 ≤ 4 级逻辑；曾把 亮度计算+RAM 读写 并在一拍 → 11 级 11.2ns）
            case (c_st)
                3'd0: begin
                    bc_reg <= base_col(c_led2);
                    bf_reg <= alm_act ? tri10({frm_cnt[8:0], 1'b0})
                                      : tri10(frm_cnt[9:0] + {c_led2, 6'd0});
                    fl_reg <= flash[c_led2];
                    wr_ad  <= c_led2;
                end
                3'd1: begin
                    if (alm_act)
                        mv_reg <= 6'd16 + {1'b0, bf_reg[7:3]};
                    else if (s_frz && wr_ad == 4'd2)
                        mv_reg <= 6'd40;                        // 定格灯常亮
                    else
                        mv_reg <= 6'd24 + {1'b0, bf_reg[7:4]}
                                + {2'b00, fl_reg[7:4]} - (bf_reg[7:4] >= 4'd8 ? 6'd8 : 6'd0);
                end
                3'd2: begin
                    if (!boot_sweep_on)
                        q_reg <= band_of(mv_reg, frm_cnt[2:0]);
                    else
                        q_reg <= 4'sd0;       // 彩虹不经缩放（级3 直接出三角波色）
                end
                3'd3: begin
                    if (boot_sweep_on)
                        rgb_word <= {tri10(sw_bt), tri10(sw_bt + 10'd341),
                                     tri10(sw_bt + 10'd682)};
                    else
                        rgb_word <= {scale_ch(bc_reg[23:16], q_reg),
                                     scale_ch(bc_reg[15:8],  q_reg),
                                     scale_ch(bc_reg[7:0],   q_reg)};
                end
                default: col_reg[wr_ad] <= rgb_word;
            endcase
            if (c_st == 3'd4) begin
                c_st   <= 3'd0;
                c_led2 <= c_led2 + 4'd1;
            end else begin
                c_st <= c_st + 3'd1;
            end
        end
        if (calc_k == 6'd49) begin
            calc_ph <= 1'b0;
            rst_cnt <= 16'd50;               // 计算已占 50 拍，从 50 继续计复位
        end else begin
            calc_k <= calc_k + 6'd1;
        end
    end else begin
        // ---- 复位等待 ----
        din <= 1'b0;
        if (rst_cnt >= RST_T[15:0]) begin
            rst_cnt <= 16'd0;
            // LED0 放最高位端（发送从 MSB 先出 → LED0 的 G7 先发，顺序正确；
            // 旧版把 LED0 打包在低位端导致灯序整体颠倒）
            for (ai = 0; ai < NLED; ai = ai + 1)
                shreg[(NLED-1-ai)*24 +: 24] <= {col_reg[ai][15:8], col_reg[ai][23:16],  // GRB
                                                col_reg[ai][7:0]};
            bit_lvl <= col_reg[0][15];           // 首位 = LED0 的 G7（G 字节 MSB）
            bit_cnt <= 24*NLED - 1;
            tick    <= 7'd0;
            sending <= 1'b1;
        end else begin
            rst_cnt <= rst_cnt + 16'd1;
        end
    end
end

endmodule
