// 外接正弦复现 TB：AD7928 逐位行为模型（复用 adc_if_tb 已验证模型）挂到
// top_sim 的 SPI 引脚，A 相注入带偏置正弦、其余通道恒 2048（采样板空载）。
// 场景化运行：每相位改变 幅度/偏置/频率，等 PLL 锁定后采 1 帧渲 PPM，
// 并导出最近 600 个采样点的 CSV（raw/dc/居中后码值/锁定/频率）供数值分析。
`timescale 1ns / 1ps

module extsig_tb;

reg clk100 = 1'b0;
reg clk74  = 1'b0;
always #5.000 clk100 = ~clk100;              // 100MHz
always #6.734 clk74  = ~clk74;               // 74.25MHz

wire        ADC_SCLK, ADC_CS_N, ADC_MOSI;
wire        vid_hs, vid_vs, vid_de;
wire [7:0]  rgb_r, rgb_g, rgb_b;
wire [7:0]  LED;
wire        WS2812_DIN, BUZZER;

reg         BTN_FUNC = 1'b1, BTN_VIEW = 1'b1, BTN_FREEZE = 1'b1;
reg         BTN_SEL1 = 1'b1, BTN_SEL2 = 1'b1, BTN_SEL3 = 1'b1;
reg         BTN_SW2 = 1'b0, BTN_SW3 = 1'b0;   // user_input 视 SW2/SW3 为高有效：空闲须为 0
reg         ENC1_CHA = 1'b1, ENC1_CHB = 1'b1, ENC1_SW = 1'b1;
reg         ENC2_CHA = 1'b1, ENC2_CHB = 1'b1;
reg         ENC3_CHA = 1'b1, ENC3_CHB = 1'b1, ENC3_SW = 1'b1;
reg         ENC4_CHA = 1'b1, ENC4_CHB = 1'b1, ENC4_SW = 1'b1;

reg        adc_dout;                        // 模型 DOUT

top_sim #(.CLK_FREQ(100_000_000)) dut (
    .sim_clk_100m(clk100), .sim_clk_74m(clk74),
    .ADC_SCLK(ADC_SCLK), .ADC_CS_N(ADC_CS_N), .ADC_MISO(adc_dout), .ADC_MOSI(ADC_MOSI),
    .vid_hs(vid_hs), .vid_vs(vid_vs), .vid_de(vid_de),
    .rgb_r(rgb_r), .rgb_g(rgb_g), .rgb_b(rgb_b),
    .LED(LED),
    .BTN_FUNC(BTN_FUNC), .BTN_VIEW(BTN_VIEW), .BTN_FREEZE(BTN_FREEZE),
    .BTN_SEL1(BTN_SEL1), .BTN_SEL2(BTN_SEL2), .BTN_SEL3(BTN_SEL3),
    .BTN_SW2(BTN_SW2), .BTN_SW3(BTN_SW3),
    .ENC1_CHA(ENC1_CHA), .ENC1_CHB(ENC1_CHB), .ENC1_SW(ENC1_SW),
    .ENC2_CHA(ENC2_CHA), .ENC2_CHB(ENC2_CHB),
    .ENC3_CHA(ENC3_CHA), .ENC3_CHB(ENC3_CHB), .ENC3_SW(ENC3_SW),
    .ENC4_CHA(ENC4_CHA), .ENC4_CHB(ENC4_CHB), .ENC4_SW(ENC4_SW),
    .WS2812_DIN(WS2812_DIN), .BUZZER(BUZZER)
);

// ===========================================================================
// AD7928 行为模型（自 adc_if_tb，去掉时序 FAIL 检查，输入改为连续时间函数）
// ===========================================================================
real  sig_amp;                               // A 相正弦幅度（码）
reg   clip_en = 0;                           // 运放输出级下钳位使能
integer clip_floor = 0;                      // 钳位底码
reg   stray_en = 0;                          // 单点野值注入（去刺器验证工具，默认关）
reg   zc_glitch_en = 0;                      // 零点双触发毛刺注入（不应期验证）
reg   harsh_en = 0;                          // V28 狠场景：全通道坏帧风暴+标记错位
integer frame_ctr = 0;
real  sig_dc;                                // A 相直流偏置（码）
real  sig_freq;                              // A 相频率（Hz）

function integer ch_input;                   // 各通道输入码（采样于 CS↓）
    input [2:0] ch;
    real t_us, ang, cyc, cycf;
    integer v;
    begin
        if (ch == 3'd0) begin
            t_us = $realtime / 1000.0;
            // ModelSim $sin 大角度单精度约化会输出伪随机值：先取小数周期再求 sin
            cyc = sig_freq * t_us * 1.0e-6;
            ang  = 6.283185307179586 * (cyc - $rtoi(cyc));
            v = $rtoi(sig_dc + sig_amp * $sin(ang));
            // 零点毛刺注入：上升/下降两个过零后 15µs 内的采样帧交替 跌破武装线/
            // 冲过触发线——复现实机"偏置节点换向踢动"（V24 不应期 12ms 验证：
            // 下降过零（~10ms）处的毛刺对在 4ms 不应期下仍触发 180° 伪事件）
            if (zc_glitch_en) begin
                cycf = (cyc - $rtoi(cyc)) * 1.0e6;       // 周期内时刻 µs
                if (cycf < 15.0 || (cycf > 10000.0 && cycf < 10015.0))
                    v = ($rtoi(cycf) % 2 == 0) ? (sig_dc - 12'd250) : (sig_dc + 12'd250);
            end
            if (v < 0)    v = 0;             // 模拟负半周削底（ADC 物理下限）
            if (v > 4095) v = 4095;          // 模拟正半周削顶
            if (clip_en && v < clip_floor) v = clip_floor;   // 运放输出级下钳位
            ch_input = v;
        end else begin
            ch_input = 2048;                 // 空载通道 = 1.25V 偏置
        end
    end
endfunction

reg  [11:0] ctrl_reg;
reg  [2:0]  conv_chan;
reg  [15:0] dout_frame;
integer     dout_bit;
integer     fall_cnt;
integer     frame_idx;
reg  [11:0] din_shift;
reg  [11:0] frame_val;

always @(negedge ADC_SCLK) if (!ADC_CS_N) begin
    fall_cnt = fall_cnt + 1;
    if (fall_cnt <= 12) begin
        din_shift = {din_shift[10:0], ADC_MOSI};
        if (fall_cnt == 12 && din_shift[11] === 1'b1) ctrl_reg = din_shift;
    end
    if (fall_cnt <= 15) begin
        #40 adc_dout = dout_frame[dout_bit];
        dout_bit = dout_bit - 1;
    end else begin
        #20 adc_dout = 1'bz;
    end
end

integer a_ctr = 0;                     // A 相帧计数（坏值注入定向到 A 相=PLL 锁相通道）
always @(negedge ADC_CS_N) begin
    fall_cnt  = 0;
    dout_bit  = 14;
    adc_dout  = 1'bz;
    frame_ctr = frame_ctr + 1;
    if (conv_chan == 3'd0) a_ctr = a_ctr + 1;
    frame_val = ch_input(conv_chan);    // 采样时刻 = 本帧 CS↓
    // 坏值注入：A 相每 60 帧（~3.1ms，相位渐移扫过全周期）一枚，0/4095 交替
    // ——低坏值（0）最致命：正半周任意相位 跌破武装线→下帧恢复冲过触发线
    if (stray_en && (a_ctr % 120) == 7  && conv_chan == 3'd0) frame_val = 12'd4095;
    if (stray_en && (a_ctr % 120) == 67 && conv_chan == 3'd0) frame_val = 12'd0;
    // V28 狠场景：任意通道每 11 帧一枚 4095/0 交替坏帧（每通道平均 3.4ms 一枚，
    // 每周期 ~6 枚）+ 每 53 帧一次通道标记错位（数据写入相邻通道寄存器——
    // 复现 SPI 位错把 B 相数据写进 A 相这类最恶劣情形）
    if (harsh_en && (frame_ctr % 11) == 3)  frame_val = 12'd4095;
    if (harsh_en && (frame_ctr % 11) == 7)  frame_val = 12'd0;
    dout_frame = {1'b0, conv_chan, frame_val};
    if (harsh_en && (frame_ctr % 53) == 31)
        dout_frame = {1'b0, conv_chan + 3'd1, frame_val};   // 标记错位
    #30 adc_dout = dout_frame[15];
end

always @(posedge ADC_CS_N) begin
    adc_dout  = 1'bz;
    conv_chan = ctrl_reg[8:6];               // 本帧控制字选通下一帧通道
    frame_idx = frame_idx + 1;
end

// ===========================================================================
// 采样数据记录：eff_sample_pulse 后 1 拍锁存 wf_va_r / ch_dc[0]，环形存最近样本
// ===========================================================================
integer  rec_wr = 0;
reg [11:0] rec_raw  [0:2047];
reg [11:0] rec_dc   [0:2047];
reg [11:0] rec_cen  [0:2047];
reg        rec_lock [0:2047];
reg [31:0] rec_freq [0:2047];

always @(posedge clk100) begin
    if (dut.eff_sample_pulse) begin
        // V26 修复：写入必须 &2047——原版 rec_wr 超过数组深度后越界写被
        // 丢弃，dump 出的 600 条永远是开场 s1 的陈旧数据（中心 2100/locked=0）
        rec_raw[rec_wr & 2047]  <= dut.ch_data[0];
        rec_dc [rec_wr & 2047]  <= dut.dc_bias_c;
        rec_cen [rec_wr & 2047] <= dut.wf_va_r;
        rec_lock[rec_wr & 2047] <= dut.pll_locked_sync;
        rec_freq[rec_wr & 2047] <= dut.freq_out;
        rec_wr <= rec_wr + 1;
    end
end



// wf 握手两侧计数（100MHz 侧逐拍采样比较）
integer tog_cnt = 0, sp_cnt = 0;
reg wf_prev = 1'b0;
always @(posedge clk100) begin
    if (dut.eff_sample_pulse) sp_cnt = sp_cnt + 1;
    if (dut.wf_toggle !== wf_prev) begin tog_cnt = tog_cnt + 1; wf_prev = dut.wf_toggle; end
end

// 显示域 wf 通路探针
integer wf_sample_cnt = 0, wf_capture_cnt = 0;
integer off_ram_x = 0;
always @(posedge clk74) begin
    if (dut.display_ctrl_inst.wf_sample) wf_sample_cnt = wf_sample_cnt + 1;
    if (dut.display_ctrl_inst.wf_capture) wf_capture_cnt = wf_capture_cnt + 1;
end

// ===========================================================================
// 帧捕获
// ===========================================================================
reg cap = 1'b0;
reg [10:0] px = 11'd0, py = 11'd0;
reg [7:0] frames = 8'd0;
reg [23:0] fb [0:921599];

always @(posedge clk74) begin
    if (vid_de) begin
        if (cap) fb[py * 1280 + px] <= {rgb_r, rgb_g, rgb_b};
        if (px == 11'd1279) begin
            px <= 11'd0;
            if (py == 11'd719) begin py <= 11'd0; frames <= frames + 8'd1; end
            else py <= py + 11'b1;
        end else px <= px + 11'b1;
    end
end

reg [8*24-1:0] dump_name;
task dump_ppm;
    integer f, i;
    begin
        f = $fopen(dump_name, "wb");
        $fwrite(f, "P6\n1280 720\n255\n");
        for (i = 0; i < 921600; i = i + 1)
            $fwrite(f, "%c%c%c", fb[i][23:16], fb[i][15:8], fb[i][7:0]);
        $fclose(f);
        $display("dumped %0s", dump_name);
    end
endtask

task capture_one;
    begin
        @(negedge clk74);
        wait (frames[0] == 1'b0);            // 对齐帧边界（wrap 后偶数计数）
        @(negedge clk74);
        wait (frames[0] == 1'b1);
        cap = 1'b1;
        wait (frames[0] == 1'b0);            // 采完整一帧
        cap = 1'b0;
        @(posedge clk74);
    end
endtask

// ===========================================================================
// 工具：按键 / 场景切换 / 等锁定 / 导出 CSV
// ===========================================================================
task press_func;
    begin
        BTN_FUNC = 1'b0; #10_000_000; BTN_FUNC = 1'b1; #1_000_000;
    end
endtask

// 等锁定（上限 150ms）；锁定后再等 extra_ms 稳定
task wait_settle(input integer extra_ms);
    integer waited;
    begin
        waited = 0;
        while (!dut.pll_locked_sync && waited < 150_000_000) begin
            #1_000_000; waited = waited + 1_000_000;
        end
        if (!dut.pll_locked_sync)
            $display("[%0t] WARN: PLL not locked after 150ms", $time);
        #(extra_ms * 1_000_000);
    end
endtask

// 导出最近 n 个采样点 + 当前读数
task dump_csv(input [8*24-1:0] nm, input integer n);
    integer f, i, idx;
    begin
        f = $fopen(nm, "w");
        $fwrite(f, "idx,raw,dc,centered,locked,freq01\n");
        for (i = 0; i < n; i = i + 1) begin
            idx = (rec_wr - n + i) & 2047;
            $fwrite(f, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                    i, rec_raw[idx], rec_dc[idx], rec_cen[idx],
                    rec_lock[idx], rec_freq[idx]);
        end
        $fclose(f);
        $display("csv %0s  rms_va=%0d rms_vb=%0d thd_va=%0d freq=%0d dc=%0d amp_param=%0f dc_param=%0f wfsmpl=%0d wfcap=%0d wf_wr=%0d tog=%0d sp=%0d recwr=%0d", 
                 nm, dut.rms_va, dut.rms_vb, dut.thd_va, dut.freq_out, dut.dc_bias_c, sig_amp, sig_dc, wf_sample_cnt, wf_capture_cnt, dut.display_ctrl_inst.wf_wr, tog_cnt, sp_cnt, rec_wr);
    end
endtask

integer scenario_ms;
initial begin
    scenario_ms = 120;                        // 每场景稳定时间
    sig_amp = 819.0; sig_dc = 2100.0; sig_freq = 50.000;
    ctrl_reg = 12'h000; conv_chan = 3'd0; adc_dout = 1'bz;
    frame_idx = 0; frame_val = 12'd0; dout_frame = 16'd0; dout_bit = 14;
    fall_cnt = 0; din_shift = 12'd0;

    #200_000;                                  // rst_counter 释放
    // 等显示文本初始化（fmt 计数 ~0.9ms）+ 自检链路跑起来
    #5_000_000;
    // 切到测量模式：按 2 次 FUNC（自检→单相A→三相四线）
    press_func; #20_000_000;
    press_func; #5_000_000;
    $display("[%0t] func_mode=%0d", $time, dut.func_mode);

    // ---- 场景 1：典型外接 ±20V（ADC ±819 码）+ 偏置 +52 码 ----
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s1_typ.ppm"; dump_ppm;
    dump_csv("ext_s1_typ.csv", 600);

    // ---- 场景 2：大幅 ±40V（±1638 码）满幅附近 ----
    sig_amp = 1638.0; sig_dc = 2048.0;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s2_big.ppm"; dump_ppm;
    dump_csv("ext_s2_big.csv", 600);

    // ---- 场景 3：偏置掉底（模拟运放偏置丢失/分压地偏移）----
    //      dc=700，amp=819 → 负半周 700-819<0 削底：正半周完好、负半周趴底
    sig_amp = 819.0; sig_dc = 700.0;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s3_clipbot.ppm"; dump_ppm;
    dump_csv("ext_s3_clipbot.csv", 600);

    // ---- 场景 4：小幅 + 偏频（±330 码，49.7Hz，噪声未建模仿靠滞回）----
    sig_amp = 330.0; sig_dc = 2100.0; sig_freq = 49.700;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s4_small.ppm"; dump_ppm;
    dump_csv("ext_s4_small.csv", 600);


    // ---- 场景 5：实测偏置 1.381V（2263 码）+ 典型幅度 ±819 码 ----
    //      FPGA 直流自适应（ch_dc）应自动居中，渲染无畸变
    sig_amp = 819.0; sig_dc = 2160.0; sig_freq = 50.000;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s5_bias1381.ppm"; dump_ppm;
    dump_csv("ext_s5_bias1381.csv", 600);

    // ---- 场景 6：偏置 1.381V + ±50V（2048 码）：正半周 4311 码削顶、负半周 215 码完好 ----
    //      演示"偏置偏高→正半周先削"：与实机"负半周坏"相反
    sig_amp = 2048.0; sig_dc = 2160.0;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s6_bias_clip_top.ppm"; dump_ppm;
    dump_csv("ext_s6_bias_clip_top.csv", 600);

    // ---- 场景 7：实机照片复刻——负半周运放钳位（底码 1710 = 显示 -13.5V）----
    sig_amp = 676.0; sig_dc = 2160.0; sig_freq = 50.000;
    clip_en = 1; clip_floor = 1710;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s7_opclip.ppm"; dump_ppm;
    dump_csv("ext_s7_opclip.csv", 600);
    clip_en = 0;

    // ---- 场景 8：零点双触发毛刺（V24 不应期板级复现验证）----
    sig_amp = 819.0; sig_dc = 2160.0; sig_freq = 50.000;
    zc_glitch_en = 1;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s8_zcglitch.ppm"; dump_ppm;
    dump_csv("ext_s8_zcglitch.csv", 600);
    zc_glitch_en = 0;

    // ---- 场景 9：随机相位单帧坏值（V25 去刺验证：每 500 帧一刺 4095 码，
    //      A 相约 26ms 一刺≈每周期一枚、相位渐移——复现实机"每周期 1~2 采样
    //      孤立毛刺"：坏帧在 PLL 任意相位完成 武装→触发 → 半周期相位误差 →
    //      PI 打满钳位 = FREQ 乱跳 + FFT 泄漏（RMS/THD 乱跳）+ 波形断崖）----
    sig_amp = 819.0; sig_dc = 2160.0; sig_freq = 50.000;
    stray_en = 1;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s9_stray.ppm"; dump_ppm;
    dump_csv("ext_s9_stray.csv", 600);
    stray_en = 0;

    // ---- 场景 10（V28 狠验证）：全通道坏帧风暴 + 通道标记错位。判据：
    //      FREQ 稳定（无 44x/54x 钳位）、两次 dump 的 rms/thd 逐轮一致（读数
    //      稳定=FFT 窗干净）、centered 相邻跳变 ≤ 纯正弦斜率+2（无断崖）----
    sig_amp = 819.0; sig_dc = 2160.0; sig_freq = 50.000;
    harsh_en = 1;
    wait_settle(scenario_ms);
    capture_one;
    dump_name = "ext_s10_harsh.ppm"; dump_ppm;
    dump_csv("ext_s10_harsh.csv", 600);
    #100_000_000;                            // 再跑 100ms 第二次 dump（读数稳定性）
    dump_csv("ext_s10_harsh_b.csv", 600);
    harsh_en = 0;

    $display("=== extsig_tb done ===");
    $finish;
end

initial begin
    #3_400_000_000;
    $display("TIMEOUT extsig_tb");
    $finish;
end

endmodule
