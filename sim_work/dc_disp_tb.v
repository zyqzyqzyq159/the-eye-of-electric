`timescale 1ns / 1ps
// 端到端直流居中渲染测试：+200 码偏置正弦 → (top.v 原文提取的) 直流估计 +
// dc_center → display_ctrl 渲染 → A 窗迹线必须以中线 154 居中（±2px）、连续、幅度正确。
// 复现用户实机症状：AD7928 前是干净正弦（示波器已确认），屏上波形偏一边且稀疏。
module dc_disp_tb;

reg clk100 = 1'b0;
always #5 clk100 = ~clk100;
reg clk74 = 1'b0;
always #6.734 clk74 = ~clk74;

reg rst_n = 1'b0;
wire [71:0] dc_flat;
reg [11:0] x_sig;                        // 当前"ADC 通道保持寄存器"值（带 +200 偏置）
reg samp_pulse;
integer pc, k;

// --- top.v 直流估计块（脚本从源文件提取，保证逐字一致）---
dc_probe dut(.clk_100m(clk100), .rst_n(rst_n), .eff_sample_pulse(samp_pulse),
             .ch_flat({48'd0, x_sig}), .dc_flat(dc_flat));
wire [11:0] ch_dc0 = dc_flat[11:0];

// 采样脉冲 12.8kHz（78µs = 7800 拍），每拍更新信号值（模拟 adc 通道保持寄存器）
always @(posedge clk100) begin
    if (pc >= 7799) begin pc <= 0; samp_pulse <= 1'b1; end
    else begin pc <= pc + 1; samp_pulse <= 1'b0; end
end
real ph;
always @(posedge clk100) if (pc == 3899) begin   // 脉冲中途更新（模拟异步 adc_valid）
    ph = 6.283185307179586 * k / 256.0;
    x_sig = 2248 + $rtoi(400.0 * $sin(ph));      // 均值 2248（+200 偏置），幅度 ±400
    k = k + 1;
end

// --- top.v 波形握手路径（测量模式分支）：wf_x <= dc_center(ch_data, ch_dc) ---
reg wf_t = 1'b0;
reg [1:0] wf_dly;
reg [11:0] wfa, wfb, wfc, wfi_a, wfi_b, wfi_c;
always @(posedge clk100 or negedge rst_n) begin
    if (!rst_n) begin wf_t <= 1'b0; wf_dly <= 2'd0; end
    else begin
        wf_dly <= {wf_dly[0], samp_pulse};
        if (samp_pulse) begin
            wfa   <= dut.dc_center(x_sig, ch_dc0);
            wfb   <= 12'd2048; wfc <= 12'd2048;
            wfi_a <= 12'd2048; wfi_b <= 12'd2048; wfi_c <= 12'd2048;
        end
        if (wf_dly[1]) wf_t <= ~wf_t;
    end
end

wire vid_hs, vid_vs, vid_de;
wire [7:0] rgb_r, rgb_g, rgb_b;
reg [31:0] rms_va = 32'd2200, rms_vb = 32'd2200, rms_vc = 32'd2200;   // → ±50V 挡
reg [31:0] rms_ia = 32'd2200, rms_ib = 32'd2200, rms_ic = 32'd2200;
reg [31:0] thd_va = 32'd0, thd_vb = 32'd0, thd_vc = 32'd0;
reg [31:0] thd_ia = 32'd0, thd_ib = 32'd0, thd_ic = 32'd0;
reg [31:0] fundamental_freq = 32'd500;
reg [239:0] harm_pct_va = 240'd0, harm_pct_vb = 240'd0, harm_pct_vc = 240'd0;
reg [239:0] harm_pct_ia = 240'd0, harm_pct_ib = 240'd0, harm_pct_ic = 240'd0;
reg [15:0] phase_ab = 16'd43691, phase_bc = 16'd43691, phase_ca = 16'd43691;
reg [15:0] phase_ia = 16'd0, phase_ib = 16'd0, phase_ic = 16'd0;
reg phase_sequence = 1'b1;
reg [31:0] unbalance_degree = 32'd0;
reg mode_t = 1'b0, freeze = 1'b0;
reg [2:0] event_alarm = 3'd0;
reg done_t = 1'b0;
reg func_t = 1'b0;
reg [10:0] func_d = 11'd0;
reg view_t = 1'b0;
reg [44:0] view_d = 45'd0;
reg [177:0] pwr_bus_d = 178'd0;

reg rst_meta, rst_px;
always @(posedge clk74 or negedge rst_n) begin
    if (!rst_n) begin rst_meta <= 1'b0; rst_px <= 1'b0; end
    else begin rst_meta <= 1'b1; rst_px <= rst_meta; end
end

display_ctrl #(
    .H_RES(1280), .V_RES(720), .DATA_BITS(12), .HARMONICS_NUM(15)
) disp (
    .clk(clk74), .rst_n(rst_px),
    .vid_hs(vid_hs), .vid_vs(vid_vs), .vid_de(vid_de),
    .rgb_r(rgb_r), .rgb_g(rgb_g), .rgb_b(rgb_b),
    .rms_va(rms_va), .rms_vb(rms_vb), .rms_vc(rms_vc),
    .rms_ia(rms_ia), .rms_ib(rms_ib), .rms_ic(rms_ic),
    .thd_va(thd_va), .thd_vb(thd_vb), .thd_vc(thd_vc),
    .thd_ia(thd_ia), .thd_ib(thd_ib), .thd_ic(thd_ic),
    .fundamental_freq(fundamental_freq),
    .harm_pct_va(harm_pct_va), .harm_pct_vb(harm_pct_vb), .harm_pct_vc(harm_pct_vc),
    .harm_pct_ia(harm_pct_ia), .harm_pct_ib(harm_pct_ib), .harm_pct_ic(harm_pct_ic),
    .phase_ab(phase_ab), .phase_bc(phase_bc), .phase_ca(phase_ca),
    .phase_ia(phase_ia), .phase_ib(phase_ib), .phase_ic(phase_ic),
    .phase_sequence(phase_sequence), .unbalance_degree(unbalance_degree),
    .done_toggle(done_t), .wf_toggle(wf_t),
    .wf_va(wfa), .wf_vb(wfb), .wf_vc(wfc),
    .wf_ia(wfi_a), .wf_ib(wfi_b), .wf_ic(wfi_c),
    .mode_toggle(mode_t), .freeze(freeze), .event_alarm(event_alarm),
    .func_toggle(func_t), .func_data(func_d),
    .view_toggle(view_t), .view_data(view_d),
    .pwr_bus(pwr_bus_d), .enable(1'b1)
);

// ---- A 窗迹线快照（帧尾双缓冲）----
reg [10:0] px = 0, py = 0;
reg [8:0]  frames = 0;
reg        fnd [0:1023];
reg [10:0] ycc [0:1023];
reg        fnd2 [0:1023];
reg [10:0] ycc2 [0:1023];
integer ci;
always @(posedge clk74) begin
    if (vid_de) begin
        if (py >= 11'd64 && py < 11'd244 && px >= 11'd216 && px < 11'd1240 &&
            rgb_r == 8'hFF && rgb_g == 8'h55 && rgb_b == 8'h55) begin
            fnd[px[9:0] - 10'd216] <= 1'b1;
            ycc[px[9:0] - 10'd216] <= py;
        end
        if (px == 11'd1279 && py == 11'd719) begin
            for (ci = 0; ci < 1024; ci = ci + 1) begin
                fnd2[ci] <= fnd[ci]; ycc2[ci] <= ycc[ci]; fnd[ci] <= 1'b0;
            end
            frames <= frames + 9'd1;
        end
        if (px == 11'd1279) begin
            px <= 11'd0;
            if (py == 11'd719) py <= 11'd0;
            else py <= py + 11'd1;
        end else px <= px + 11'd1;
    end
end

integer errors;

initial begin
    k = 0; pc = 0; errors = 0; x_sig = 12'd2248;
    rst_n = 1'b0;
    #133.4;
    rst_n = 1'b1;
    #900000;
    @(negedge clk100); done_t = ~done_t; repeat (4) @(negedge clk74);
    done_t = ~done_t;        repeat (4) @(negedge clk74);

    // 等 5 帧（DC 估计 20ms 收敛 + 波形环 40ms 刷新 + 渲染）
    begin : waitf
        integer f0;
        f0 = frames;
        wait (frames == f0 + 9'd5);
    end
    @(negedge clk74);
    begin : an
        integer c2, n, ysum, mn, mx, miss;
        n = 0; ysum = 0; mn = 2048; mx = 0; miss = 0;
        for (c2 = 0; c2 < 1024; c2 = c2 + 1) begin
            if (fnd2[c2]) begin
                n = n + 1; ysum = ysum + ycc2[c2];
                if (ycc2[c2] < mn) mn = ycc2[c2];
                if (ycc2[c2] > mx) mx = ycc2[c2];
            end
        end
        $display("ch_dc = %0d (expect 2248)", ch_dc0);
        if (ch_dc0 >= 2246 && ch_dc0 <= 2250) $display("PASS: DC estimate");
        else begin $display("FAIL: DC estimate"); errors = errors + 1; end
        $display("trace: mean y = %0d (mid 154), pp = %0d px, cols = %0d/1024",
                 ysum/n, mx-mn, n);
        if (ysum/n >= 152 && ysum/n <= 156) $display("PASS: trace centered on midline");
        else begin $display("FAIL: not centered"); errors = errors + 1; end
        if (mx-mn >= 30 && mx-mn <= 42) $display("PASS: amplitude ~35px");
        else begin $display("FAIL: amplitude"); errors = errors + 1; end
        if (n >= 920) $display("PASS: trace continuous (no gaps)");
        else begin $display("FAIL: only %0d columns", n); errors = errors + 1; end
    end
    if (errors == 0) $display("=== dc_disp_tb ALL PASS ===");
    else             $display("=== dc_disp_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #300_000_000;
    $display("TIMEOUT dc_disp_tb");
    $finish;
end

endmodule
