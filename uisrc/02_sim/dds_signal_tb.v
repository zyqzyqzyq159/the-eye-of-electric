`timescale 1ns / 1ps
// dds_signal 模块仿真：三相 120° DDS（LUT_SIZE=256 相位截断 + 32bit 相位累加器）
// 验证幅度范围、三相相位差、频率切换、失真注入（DIP1）与使能控制
module dds_signal_tb;

reg         clk;
reg         rst_n;
reg  [31:0] freq_config;
reg  [15:0] amp_config;
reg         enable;
reg         select;
reg         distort;

wire [11:0] dds_va;
wire [11:0] dds_vb;
wire [11:0] dds_vc;
wire        dds_valid;

parameter PERIOD = 10;
always #(PERIOD/2) clk = ~clk;

dds_signal #(
    .CLK_FREQ       (100_000_000),
    .DAC_BITS       (12),
    .LUT_SIZE       (256)
) uut (
    .clk         (clk),
    .rst_n       (rst_n),
    .dds_va      (dds_va),
    .dds_vb      (dds_vb),
    .dds_vc      (dds_vc),
    .dds_valid   (dds_valid),
    .freq_config (freq_config),
    .amp_config  (amp_config),
    .enable      (enable),
    .select      (select),
    .distort_en  (distort)
);

// 采样分频可变：测试 1/2 用 2048（48.8kHz，2000 点约 2 个周期，看幅度/过零）；
// 测试 3 用 7813（12.8kHz，与板上采样率一致，512 点 = 恰好 2 个基波周期，
// 整数 bin 相关法测谐波，无频谱泄漏）
reg [12:0] samp_div;
reg [11:0] samp_va [0:2047];
reg [11:0] samp_vb [0:2047];
reg [11:0] samp_vc [0:2047];
integer    samp_n;
reg [12:0] samp_cnt;

always @(posedge clk) begin
    if (samp_cnt >= samp_div) samp_cnt <= 13'd0;
    else samp_cnt <= samp_cnt + 1'b1;
end

wire samp_tick = (samp_cnt == samp_div);

always @(posedge clk) begin
    if (samp_tick && dds_valid && samp_n < 2048) begin
        samp_va[samp_n] <= dds_va;
        samp_vb[samp_n] <= dds_vb;
        samp_vc[samp_n] <= dds_vc;
        samp_n <= samp_n + 1'b1;
    end
end

// 过零检测：先找 A 的首个上升过零，再在其后找 B/C 的过零
// （采集起点相位任意，直接比较三通道首个过零顺序会因相位回绕误判）
integer x_a, x_b, x_c;

task find_crossings;
    integer n;
    begin
        x_a = -1; x_b = -1; x_c = -1;
        for (n = 1; n < samp_n; n = n + 1)
            if (x_a < 0 && samp_va[n-1] < 12'd2048 && samp_va[n] >= 12'd2048) x_a = n;
        for (n = x_a + 1; n < samp_n; n = n + 1) begin
            if (x_b < 0 && samp_vb[n-1] < 12'd2048 && samp_vb[n] >= 12'd2048) x_b = n;
            if (x_c < 0 && samp_vc[n-1] < 12'd2048 && samp_vc[n] >= 12'd2048) x_c = n;
        end
    end
endtask

reg [11:0] va_max, va_min, vb_max, vb_min, vc_max, vc_min;
integer    n;

task update_range;
    begin
        for (n = 0; n < samp_n; n = n + 1) begin
            if (samp_va[n] > va_max) va_max = samp_va[n];
            if (samp_va[n] < va_min) va_min = samp_va[n];
            if (samp_vb[n] > vb_max) vb_max = samp_vb[n];
            if (samp_vb[n] < vb_min) vb_min = samp_vb[n];
            if (samp_vc[n] > vc_max) vc_max = samp_vc[n];
            if (samp_vc[n] < vc_min) vc_min = samp_vc[n];
        end
    end
endtask

integer phase_ab_est, phase_bc_est;

initial begin
    clk         = 1'b0;
    rst_n       = 1'b0;
    freq_config = 32'd50;
    amp_config  = 16'd2048;
    enable      = 1'b0;
    select      = 1'b0;
    distort     = 1'b0;
    samp_n      = 0;
    samp_cnt    = 13'd0;
    samp_div    = 13'd2047;
    va_max = 12'd0; va_min = 12'hFFF;
    vb_max = 12'd0; vb_min = 12'hFFF;
    vc_max = 12'd0; vc_min = 12'hFFF;

    #1000;
    rst_n  = 1'b1;
    enable = 1'b1;
    select = 1'b1;
    #10000;

    $display("--- Test 1: 50Hz 3-phase sine, capture ~2 cycles ---");
    samp_n = 0;
    freq_config = 32'd50;
    amp_config  = 16'd2048;
    // 2000 个采样点 @48.8kHz 约 2 个周期
    wait (samp_n >= 2000);
    #1000;

    update_range;
    $display("VA range: %0d..%0d", va_min, va_max);
    $display("VB range: %0d..%0d", vb_min, vb_max);
    $display("VC range: %0d..%0d", vc_min, vc_max);

    if (va_max > 12'd3500 && va_min < 12'd600)
        $display("PASS: VA amplitude near full swing");
    else
        $display("FAIL: VA range wrong");
    if (vb_max > 12'd3500 && vc_max > 12'd3500)
        $display("PASS: VB/VC amplitudes match VA");
    else
        $display("FAIL: channel amplitude mismatch");

    find_crossings;
    $display("Rising crossings: A@%0d B@%0d C@%0d (samples)", x_a, x_b, x_c);
    if (x_a >= 0 && x_b > x_a && x_c > x_b) begin
        phase_ab_est = (x_b - x_a) * 360 / 977;   // 50Hz: 977 点/周期
        phase_bc_est = (x_c - x_b) * 360 / 977;
        $display("Estimated phase: A-B=%0ddeg B-C=%0ddeg (expect ~120)", phase_ab_est, phase_bc_est);
        if (phase_ab_est > 100 && phase_ab_est < 140 && phase_bc_est > 100 && phase_bc_est < 140)
            $display("PASS: 120deg phase separation");
        else
            $display("FAIL: phase separation wrong");
    end else
        $display("FAIL: crossing detection incomplete");

    $display("--- Test 2: Frequency change to 25Hz ---");
    samp_n = 0;
    va_max = 12'd0; va_min = 12'hFFF;
    freq_config = 32'd25;
    wait (samp_n >= 2000);
    #1000;
    update_range;
    $display("25Hz VA range: %0d..%0d", va_min, va_max);
    if (va_max > 12'd3500 && va_min < 12'd600)
        $display("PASS: 25Hz output normal");
    else
        $display("FAIL: 25Hz output wrong");

    // ======== 测试 3：失真注入（DIP1=1）：基波 + 20% 三次 + 10% 五次 ========
    // 期望：h3/基波 = 0.200，h5/基波 = 0.100（谐波表从基波表按 1/5、1/10 派生）
    // 合成峰值 ≈ 0.9 × 基波峰值（θ=90° 时 sin3θ 反相抵消）→ 码值约 205..3891
    $display("--- Test 3: Distortion injection (DIP1=1): 20%% 3rd + 10%% 5th ---");
    freq_config = 32'd50;
    amp_config  = 16'd2048;
    distort     = 1'b1;
    samp_div    = 13'd7812;      // 12.8kHz 采样
    #200003;                     // 频率切换后等约 10 个基波周期稳定
    samp_n = 0;
    va_max = 12'd0; va_min = 12'hFFF;
    vb_max = 12'd0; vb_min = 12'hFFF;
    vc_max = 12'd0; vc_min = 12'hFFF;
    wait (samp_n >= 512);        // 512 点 = 恰好 2 个基波周期
    #1000;

    update_range;
    $display("Distorted ranges: A %0d..%0d  B %0d..%0d  C %0d..%0d (expect ~205..3891, no clip)",
             va_min, va_max, vb_min, vb_max, vc_min, vc_max);
    if (va_min > 12'd150 && va_max < 12'd3950 &&
        vb_min > 12'd150 && vb_max < 12'd3950 &&
        vc_min > 12'd150 && vc_max < 12'd3950)
        $display("PASS: distorted waveform in range, no saturation");
    else
        $display("FAIL: distorted waveform clipped or out of range");

    // 整数 bin 相关法测谐波（独立于 measurement 模块的交叉验证）：
    // 512 点 / 2 周期 → 基波 = bin2，k 次谐波 = bin 2k
    begin : dft_blk
        real re1, im1, re2, im2, re3, im3, re4, im4, re5, im5, om, dc, sv;
        real m1, m2, m3, m4, m5;
        integer k;
        re1 = 0.0; im1 = 0.0; re2 = 0.0; im2 = 0.0; re3 = 0.0; im3 = 0.0;
        re4 = 0.0; im4 = 0.0; re5 = 0.0; im5 = 0.0; dc = 0.0;
        for (k = 0; k < 512; k = k + 1) begin
            // 注意必须实数减法：samp_va[k]-2048 为无符号回绕，负样点会变 4.29e9
            dc = dc + samp_va[k];
            sv = samp_va[k] - 2048.0;
            om = 6.283185307179586 * k / 256.0;   // 基波角频率 2π·2k/512
            re1 = re1 + sv * $cos(om);
            im1 = im1 + sv * $sin(om);
            re2 = re2 + sv * $cos(2.0*om);
            im2 = im2 + sv * $sin(2.0*om);
            re3 = re3 + sv * $cos(3.0*om);
            im3 = im3 + sv * $sin(3.0*om);
            re4 = re4 + sv * $cos(4.0*om);
            im4 = im4 + sv * $sin(4.0*om);
            re5 = re5 + sv * $cos(5.0*om);
            im5 = im5 + sv * $sin(5.0*om);
        end
        m1 = $sqrt(re1*re1 + im1*im1);
        m2 = $sqrt(re2*re2 + im2*im2);
        m3 = $sqrt(re3*re3 + im3*im3);
        m4 = $sqrt(re4*re4 + im4*im4);
        m5 = $sqrt(re5*re5 + im5*im5);
        $display("DFT magnitudes (codes): fund=%0.0f h2=%0.0f h3=%0.0f h4=%0.0f h5=%0.0f",
                 m1, m2, m3, m4, m5);
        $display("Harmonic ratios: h3=%.3f h5=%.3f (expect 0.200/0.100)  h2=%.3f h4=%.3f (expect ~0)  dc=%.1f",
                 m3/m1, m5/m1, m2/m1, m4/m1, dc/512.0);
        if (m3/m1 > 0.190 && m3/m1 < 0.210 &&
            m5/m1 > 0.090 && m5/m1 < 0.110 &&
            m2/m1 < 0.010 && m4/m1 < 0.010)
            $display("PASS: distortion injection 20%% 3rd + 10%% 5th verified");
        else
            $display("FAIL: harmonic ratios deviate from injected distortion");
    end

    $display("--- Test 4: Disable output ---");
    enable = 1'b0;
    #10000;
    $display("Disabled: dds_valid=%b dds_va=%0d", dds_valid, dds_va);
    if (!dds_valid)
        $display("PASS: dds_valid deasserted when disabled");
    else
        $display("FAIL: dds_valid still high");

    $display("--- All dds_signal tests done ---");
    $finish;
end

initial begin
    #500_000_000;
    $display("TIMEOUT: dds_signal_tb");
    $finish;
end

endmodule
