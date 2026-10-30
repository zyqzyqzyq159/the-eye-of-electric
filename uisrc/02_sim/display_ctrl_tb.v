`timescale 1ns / 1ps
// display_ctrl 行为仿真：720P@60 时序 + 三通道示波器界面
// 验证：H_TOTAL=1650 / V_TOTAL=750 / 1280x720 有效区，
//       顶栏/左面板/主区域绘制，SW2 模式切换（波形/频谱/谐波表），报警闪烁
module display_ctrl_tb;

reg         clk;
reg         rst_n;
reg  [31:0] rms_va, rms_vb, rms_vc;
reg  [31:0] thd_va, thd_vb, thd_vc;
reg  [31:0] thd_ia, thd_ib, thd_ic;
reg  [31:0] rms_ia, rms_ib, rms_ic;
reg [239:0] harm_pct_ia, harm_pct_ib, harm_pct_ic;
reg  [15:0] phase_ia, phase_ib, phase_ic;
reg  [31:0] fundamental_freq;
reg  [239:0] harm_pct_va, harm_pct_vb, harm_pct_vc;
reg  [15:0] phase_ab, phase_bc, phase_ca;
reg         phase_sequence;
reg  [31:0] unbalance_degree;
reg         done_toggle;
reg         wf_toggle;
reg  [11:0] wf_va, wf_vb, wf_vc;
reg  [11:0] wf_ia, wf_ib, wf_ic;
reg         mode_toggle;
reg         freeze;
reg  [2:0]  event_alarm;
reg         enable;

wire        vid_hs, vid_vs, vid_de;
wire [7:0]  rgb_r, rgb_g, rgb_b;

parameter CLK_PERIOD = 13.468;   // 74.25MHz

initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

display_ctrl #(
    .H_RES        (1280),
    .V_RES        (720),
    .DATA_BITS    (12),
    .HARMONICS_NUM(15)
) uut (
    .clk              (clk),
    .rst_n            (rst_n),
    .vid_hs           (vid_hs),
    .vid_vs           (vid_vs),
    .vid_de           (vid_de),
    .rgb_r            (rgb_r),
    .rgb_g            (rgb_g),
    .rgb_b            (rgb_b),
    .rms_va           (rms_va),
    .rms_vb           (rms_vb),
    .rms_vc           (rms_vc),
    .thd_va           (thd_va),
    .thd_vb           (thd_vb),
    .thd_vc           (thd_vc),
    .fundamental_freq (fundamental_freq),
    .harm_pct_va      (harm_pct_va),
    .harm_pct_vb      (harm_pct_vb),
    .harm_pct_vc      (harm_pct_vc),
    .phase_ab         (phase_ab),
    .phase_bc         (phase_bc),
    .phase_ca         (phase_ca),
    .phase_sequence   (phase_sequence),
    .unbalance_degree (unbalance_degree),
    .done_toggle      (done_toggle),
    .wf_toggle        (wf_toggle),
    .wf_va            (wf_va),
    .wf_vb            (wf_vb),
    .wf_vc            (wf_vc),
    .wf_ia            (wf_ia),
    .wf_ib            (wf_ib),
    .wf_ic            (wf_ic),
    .rms_ia           (rms_ia),
    .rms_ib           (rms_ib),
    .rms_ic           (rms_ic),
    .thd_ia           (thd_ia),
    .thd_ib           (thd_ib),
    .thd_ic           (thd_ic),
    .harm_pct_ia      (harm_pct_ia),
    .harm_pct_ib      (harm_pct_ib),
    .harm_pct_ic      (harm_pct_ic),
    .phase_ia         (phase_ia),
    .phase_ib         (phase_ib),
    .phase_ic         (phase_ic),
    .mode_toggle      (mode_toggle),
    .freeze           (freeze),
    .event_alarm      (event_alarm),
    .func_toggle      (1'b0),
    .func_data        (8'd0),
    .view_toggle      (1'b0),
    .view_data        (45'd0),
    .enable           (enable)
);

// ------------------------------------------------------------------
// 波形激励：正弦三相互差 120°，每 100 个像素时钟采样一次，
// 数据更新 2 拍后才翻转 wf_toggle（与 top.v 的时序约定一致）
// ------------------------------------------------------------------
real    pi_val;
reg [11:0] sine_tab [0:255];
integer si;

initial begin
    pi_val = 3.14159265;
    for (si = 0; si < 256; si = si + 1)
        sine_tab[si] = $rtoi(2048.0 + 1500.0 * $sin(2.0 * pi_val * si / 256.0));
end

reg [7:0] samp_div;
reg [1:0] wf_phase;
reg [7:0] samp_idx;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        samp_div  <= 8'd0;
        wf_phase  <= 2'd0;
        samp_idx  <= 8'd0;
        wf_toggle  <= 1'b0;
        wf_va <= 12'd2048; wf_vb <= 12'd2048; wf_vc <= 12'd2048;
    end else begin
        if (samp_div >= 8'd99) samp_div <= 8'd0;
        else samp_div <= samp_div + 1'b1;

        case (wf_phase)
            2'd0: if (samp_div == 8'd99) wf_phase <= 2'd1;
            2'd1: begin
                wf_va <= sine_tab[samp_idx];
                wf_vb <= sine_tab[samp_idx - 8'd85];   // 滞后 120deg
                wf_vc <= sine_tab[samp_idx - 8'd171];  // 滞后 240deg
                samp_idx <= samp_idx + 8'd1;
                wf_phase <= 2'd2;
            end
            2'd2: begin
                wf_toggle <= ~wf_toggle;
                wf_phase  <= 2'd0;
            end
            default: wf_phase <= 2'd0;
        endcase
    end
end

// ------------------------------------------------------------------
// 帧扫描统计：像素坐标重构 + 区域非零像素计数
// ------------------------------------------------------------------
integer tb_x, tb_y;
reg     prev_de, prev_vs;

integer top_px, panel_px, main_px, alarm_px;
integer wf_a_px, wf_b_px, wf_c_px;   // 三相波形颜色像素（A红/B绿/C蓝）
integer stop_red_px, run_grn_px;     // 顶栏 RUN/STOP 指示（定格=红 STOP / 运行=绿 RUN）
integer de_in_line, de_max, pix_in_frame, lines_in_frame;

always @(posedge clk) begin
    prev_de <= vid_de;
    prev_vs <= vid_vs;

    if (vid_de) begin
        tb_x <= tb_x + 1;
        de_in_line  <= de_in_line + 1;
        pix_in_frame <= pix_in_frame + 1;
        if (rgb_r != 0 || rgb_g != 0 || rgb_b != 0) begin
            if (tb_y < 32)                    top_px   <= top_px + 1;
            else if (tb_x < 208)              panel_px <= panel_px + 1;
            else                              main_px  <= main_px + 1;
            if (tb_y < 32 && rgb_r > 8'd200 && rgb_g < 8'd50 && rgb_b < 8'd50)
                alarm_px <= alarm_px + 1;
        end
        // 三相波形轨迹像素（窗口内按波形专色统计）
        if (tb_x >= 216 && tb_y >= 64 && tb_y < 244 &&
            rgb_r > 8'd200 && rgb_g < 8'd120 && rgb_b < 8'd120)
            wf_a_px <= wf_a_px + 1;
        if (tb_x >= 216 && tb_y >= 292 && tb_y < 472 &&
            rgb_g > 8'd200 && rgb_r < 8'd120 && rgb_b < 8'd120)
            wf_b_px <= wf_b_px + 1;
        if (tb_x >= 216 && tb_y >= 520 && tb_y < 700 &&
            rgb_b > 8'd200 && rgb_r < 8'd120 && rgb_g < 8'd120)
            wf_c_px <= wf_c_px + 1;
        // 顶栏 RUN/STOP 指示区（x1088..1120，行 0..15）
        if (tb_y < 16 && tb_x >= 1088 && tb_x < 1120) begin
            if (rgb_r > 8'd200 && rgb_g < 8'd150 && rgb_b < 8'd150)
                stop_red_px <= stop_red_px + 1;
            if (rgb_g > 8'd200 && rgb_r < 8'd150 && rgb_b < 8'd150)
                run_grn_px  <= run_grn_px + 1;
        end
    end else if (prev_de) begin
        de_max <= de_in_line;      // 锁存刚结束一行的 DE 计数
        de_in_line <= 0;
        lines_in_frame <= lines_in_frame + 1;
        tb_x <= 0;
        tb_y <= tb_y + 1;
    end

    if (vid_vs && !prev_vs) begin
        tb_y <= 0;
        lines_in_frame <= 0;
        pix_in_frame   <= 0;
    end
end

task clear_stats;
    begin
        top_px = 0; panel_px = 0; main_px = 0; alarm_px = 0;
        wf_a_px = 0; wf_b_px = 0; wf_c_px = 0;
        stop_red_px = 0; run_grn_px = 0;
    end
endtask

// 等待一个完整帧（vsync 上升沿到上升沿），期间统计
task scan_one_frame;
    begin
        @(posedge vid_vs);
        clear_stats;
        @(posedge vid_vs);
    end
endtask

// ------------------------------------------------------------------
// 主流程
// ------------------------------------------------------------------
integer i;
integer wr_ptr_1, wr_ptr_2;

initial begin
    rst_n            = 1'b0;
    // RMS=220.0V(0.1V 单位)，THD=5.0%(千分比 50)，频率 50Hz
    rms_va = 32'd2200; rms_vb = 32'd2200; rms_vc = 32'd2200;
    rms_ia = 32'd0; rms_ib = 32'd0; rms_ic = 32'd0;
    thd_ia = 32'd0; thd_ib = 32'd0; thd_ic = 32'd0;
    harm_pct_ia = 240'd0; harm_pct_ib = 240'd0; harm_pct_ic = 240'd0;
    phase_ia = 16'd0; phase_ib = 16'd0; phase_ic = 16'd0;
    wf_ia = 12'd2048; wf_ib = 12'd2048; wf_ic = 12'd2048;
    thd_va = 32'd50;   thd_vb = 32'd50;   thd_vc = 32'd50;
    fundamental_freq = 32'd50;
    harm_pct_va = 240'd0; harm_pct_vb = 240'd0; harm_pct_vc = 240'd0;
    harm_pct_va[15:0]    = 16'd1000;  // 基波 100.0%
    harm_pct_va[47:32]   = 16'd200;   // 3 次谐波 20.0%
    harm_pct_va[79:64]   = 16'd80;    // 5 次谐波 8.0%
    harm_pct_vb[15:0]    = 16'd1000;
    harm_pct_vc[15:0]    = 16'd1000;
    phase_ab = 16'd43691; phase_bc = 16'd43691; phase_ca = 16'd43691;
    phase_sequence   = 1'b1;
    unbalance_degree = 32'd0;
    done_toggle = 1'b0;
    mode_toggle = 1'b0;
    freeze      = 1'b0;
    event_alarm = 3'd0;
    enable      = 1'b1;
    tb_x = 0; tb_y = 0; prev_de = 0; prev_vs = 0;
    de_in_line = 0; pix_in_frame = 0; lines_in_frame = 0;

    #1000;
    rst_n = 1'b1;

    // 等格式化状态机初始化完成（fmt_init_cnt 到 65536）+ 波形缓冲填充
    #(CLK_PERIOD * 80_000);

    // 模拟一轮测量完成（面板数值刷新）
    done_toggle = 1'b1;
    #(CLK_PERIOD * 8);
    done_toggle = 1'b0;

    $display("--- Test 1: 720P timing ---");
    scan_one_frame;
    $display("DE pixels per line (latched): %0d (expect 1280)", de_max);
    $display("DE lines per frame: %0d (expect 720)", lines_in_frame);
    $display("DE pixels per frame: %0d (expect 921600)", pix_in_frame);

    if (de_max == 1280)
        $display("PASS: 1280 pixels per line");
    else
        $display("FAIL: line pixel count = %0d", de_max);

    if (lines_in_frame == 720)
        $display("PASS: 720 lines per frame");
    else
        $display("FAIL: line count = %0d", lines_in_frame);

    if (pix_in_frame == 921600)
        $display("PASS: 921600 active pixels per frame");
    else
        $display("FAIL: active pixel count = %0d", pix_in_frame);

    $display("--- Test 2: Mode 0 waveform UI regions ---");
    $display("Nonzero pixels: top=%0d panel=%0d main=%0d", top_px, panel_px, main_px);
    if (top_px > 100)
        $display("PASS: top bar text rendered");
    else
        $display("FAIL: top bar empty");
    if (panel_px > 100)
        $display("PASS: left panel text rendered");
    else
        $display("FAIL: left panel empty");
    if (main_px > 1000)
        $display("PASS: waveform area drawn");
    else
        $display("FAIL: waveform area empty");

    $display("--- Test 2b: three-phase waveform traces ---");
    $display("Waveform trace pixels: A=%0d B=%0d C=%0d", wf_a_px, wf_b_px, wf_c_px);
    if (wf_a_px > 100 && wf_b_px > 100 && wf_c_px > 100)
        $display("PASS: A/B/C waveform all drawn (2x3 column layout)");
    else
        $display("FAIL: missing trace A=%0d B=%0d C=%0d", wf_a_px, wf_b_px, wf_c_px);

    $display("--- Test 3: Mode 1 spectrum ---");
    mode_toggle = 1'b1;
    #(CLK_PERIOD * 8);
    mode_toggle = 1'b0;
    scan_one_frame;
    $display("Nonzero pixels: top=%0d panel=%0d main=%0d", top_px, panel_px, main_px);
    if (main_px > 1000)
        $display("PASS: spectrum bars drawn");
    else
        $display("FAIL: spectrum area empty");

    $display("--- Test 4: Mode 2 harmonic table ---");
    mode_toggle = 1'b1;
    #(CLK_PERIOD * 8);
    mode_toggle = 1'b0;
    scan_one_frame;
    $display("Nonzero pixels: top=%0d panel=%0d main=%0d", top_px, panel_px, main_px);
    if (main_px > 1000)
        $display("PASS: harmonic table drawn");
    else
        $display("FAIL: table area empty");

    $display("--- Test 5: Mode wrap back to 0 ---");
    mode_toggle = 1'b1;
    #(CLK_PERIOD * 8);
    mode_toggle = 1'b0;
    scan_one_frame;
    $display("Nonzero pixels: top=%0d panel=%0d main=%0d", top_px, panel_px, main_px);
    if (main_px > 1000)
        $display("PASS: back to waveform mode");
    else
        $display("FAIL: waveform area empty after wrap");


    $display("--- Test 6: SW3 freeze (RUN/STOP) ---");
    // 上一帧统计为运行态：应显示绿色 RUN，无红色 STOP
    if (run_grn_px > 0 && stop_red_px == 0)
        $display("PASS: RUN indicator green while running");
    else
        $display("FAIL: run_grn=%0d stop_red=%0d (expect green only)", run_grn_px, stop_red_px);

    // 定格：波形缓冲写指针应停止推进（激励仍在产生 wf_toggle 翻转）
    wr_ptr_1 = uut.wf_wr;
    freeze = 1'b1;
    #(CLK_PERIOD * 20_000);
    wr_ptr_2 = uut.wf_wr;
    if (wr_ptr_1 == wr_ptr_2)
        $display("PASS: waveform buffer write pointer frozen");
    else
        $display("FAIL: wf_wr moved %0d -> %0d while frozen", wr_ptr_1, wr_ptr_2);

    // 定格期间送入新一轮测量结果（RMS 改 110.0V），面板锁存值应保持 220.0V
    rms_va = 32'd1100;
    done_toggle = 1'b1;
    #(CLK_PERIOD * 8);
    done_toggle = 1'b0;
    #(CLK_PERIOD * 100);
    if (uut.rms_px[0] == 16'd2200)
        $display("PASS: measurement latch frozen (rms_px keeps 2200)");
    else
        $display("FAIL: rms_px=%0d leaked while frozen", uut.rms_px[0]);

    // 定格帧：仍显示三相波形 + 红色 STOP 指示
    scan_one_frame;
    $display("Frozen frame traces: A=%0d B=%0d C=%0d", wf_a_px, wf_b_px, wf_c_px);
    if (wf_a_px > 100 && wf_b_px > 100 && wf_c_px > 100)
        $display("PASS: waveform still displayed while frozen");
    else
        $display("FAIL: waveform lost while frozen A=%0d B=%0d C=%0d", wf_a_px, wf_b_px, wf_c_px);
    if (stop_red_px > 0 && run_grn_px == 0)
        $display("PASS: STOP indicator red while frozen");
    else
        $display("FAIL: stop_red=%0d run_grn=%0d (expect red only)", stop_red_px, run_grn_px);

    // 解除定格：写指针恢复推进，测量锁存恢复刷新（当前 rms=1100 应被打入）
    freeze = 1'b0;
    #(CLK_PERIOD * 20_000);
    if (uut.wf_wr != wr_ptr_2)
        $display("PASS: waveform buffer resumes after unfreeze");
    else
        $display("FAIL: wf_wr still stuck after unfreeze");
    done_toggle = 1'b1;
    #(CLK_PERIOD * 8);
    done_toggle = 1'b0;
    #(CLK_PERIOD * 100);
    if (uut.rms_px[0] == 16'd1100)
        $display("PASS: measurement latch updates after unfreeze");
    else
        $display("FAIL: rms_px=%0d not refreshed after unfreeze", uut.rms_px[0]);
    rms_va = 32'd2200;

    $display("--- Test 7: Alarm indicator ---");
    event_alarm = 3'b001;
    // 报警闪烁半周期约 225ms（2^24/74.25MHz），等亮相位再扫帧
    wait (uut.blink_cnt[24] === 1'b1);
    scan_one_frame;
    $display("Alarm pixels (red, top bar): %0d", alarm_px);
    if (alarm_px > 0)
        $display("PASS: alarm indicator rendered");
    else
        $display("FAIL: no alarm pixels");
    event_alarm = 3'b000;

    $display("--- All display_ctrl tests done ---");
    $finish;
end

initial begin
    #1_000_000_000;
    $display("TIMEOUT: display_ctrl_tb");
    $finish;
end

endmodule
