`timescale 1ns / 1ps
// measurement ??????????mag_valid + bin_in + fft_channel??
// ?????128 bin + fft_done ?????????????????
// ???RMS=0.1V?THD/?????/????=?????? 16bit(65536=360deg)
module measurement_tb;

reg         clk;
reg         rst_n;
reg  [31:0] mag_in;
reg  [15:0] phase_in;
reg  [7:0]  bin_in;
reg         mag_valid;
reg         fft_done;
reg  [2:0]  fft_channel;
reg         enable;

wire [31:0] fundamental_freq;
wire [31:0] rms_va, rms_vb, rms_vc;
wire [31:0] thd_va, thd_vb, thd_vc;
wire [239:0] harm_pct_va, harm_pct_vb, harm_pct_vc;
wire [15:0] phase_ab, phase_bc, phase_ca;
wire        phase_sequence;
wire [31:0] unbalance_degree;
wire        measure_done;
wire        done_toggle;

parameter PERIOD = 10;
always #(PERIOD/2) clk = ~clk;

measurement #(
    .FFT_SIZE     (256),
    .HARMONICS_NUM(15),
    .RMS_MUL_V    (93),
    .RMS_MUL_I    (93),
    .RMS_SHIFT    (14)
) uut (
    .clk              (clk),
    .rst_n            (rst_n),
    .mag_in           (mag_in),
    .phase_in         (phase_in),
    .bin_in           (bin_in),
    .mag_valid        (mag_valid),
    .fft_done         (fft_done),
    .fft_channel      (fft_channel),
    .fundamental_freq (fundamental_freq),
    .rms_va           (rms_va),
    .rms_vb           (rms_vb),
    .rms_vc           (rms_vc),
    .thd_va           (thd_va),
    .thd_vb           (thd_vb),
    .thd_vc           (thd_vc),
    .harm_pct_va      (harm_pct_va),
    .harm_pct_vb      (harm_pct_vb),
    .harm_pct_vc      (harm_pct_vc),
    .phase_ab         (phase_ab),
    .phase_bc         (phase_bc),
    .phase_ca         (phase_ca),
    .phase_sequence   (phase_sequence),
    .unbalance_degree (unbalance_degree),
    .measure_done     (measure_done),
    .done_toggle      (done_toggle),
    .enable           (enable)
);

// ???????bin1=???bin3=3??????? 0
task send_round(input [2:0] ch, input [31:0] m1, input [31:0] m3,
                input [15:0] ph1);
    integer b;
    begin
        fft_channel = ch;
        for (b = 0; b < 128; b = b + 1) begin
            @(posedge clk);
            mag_in    = 32'd0;
            phase_in  = 16'd0;
            bin_in    = b[7:0];
            mag_valid = 1'b1;
            if (b == 1) begin mag_in = m1; phase_in = ph1; end
            if (b == 3) begin mag_in = m3; end
        end
        @(posedge clk);
        mag_valid = 1'b0;
        mag_in    = 32'd0;
        fft_done  = 1'b1;
        @(posedge clk);
        fft_done  = 1'b0;
    end
endtask

task wait_measure_done;
    integer w;
    begin
        w = 0;
        while (!measure_done && w < 200_000) begin
            @(posedge clk);
            w = w + 1;
        end
        @(posedge clk);
    end
endtask

integer wait_cnt;

initial begin
    clk        = 1'b0;
    rst_n      = 1'b0;
    mag_in     = 32'd0;
    phase_in   = 16'd0;
    bin_in     = 8'd0;
    mag_valid  = 1'b0;
    fft_done   = 1'b0;
    fft_channel= 3'd0;
    enable     = 1'b0;

    #1000;
    rst_n  = 1'b1;
    enable = 1'b1;
    #10000;

    $display("--- Test 1: No signal (all bins zero) ---");
    send_round(3'd0, 32'd0, 32'd0, 16'd0);
    wait_measure_done;
    send_round(3'd1, 32'd0, 32'd0, 16'd0);
    wait_measure_done;
    send_round(3'd2, 32'd0, 32'd0, 16'd0);
    wait_measure_done;
    $display("  rms A/B/C = %0d/%0d/%0d (expect 0)", rms_va, rms_vb, rms_vc);
    $display("  thd A/B/C = %0d/%0d/%0d (expect 0)", thd_va, thd_vb, thd_vc);
    if (rms_va == 0 && thd_va == 0 && rms_vc == 0)
        $display("PASS: no-signal protection (zeros, no divide)");
    else
        $display("FAIL: no-signal outputs not zero");

    $display("--- Test 2: Balanced 3-phase, B/C lag A by 120/240 deg ---");
    send_round(3'd0, 32'd1000000, 32'd0, 16'd0);
    wait_measure_done;
    send_round(3'd1, 32'd1000000, 32'd0, 16'd43691);  // -21845 = -120deg
    wait_measure_done;
    send_round(3'd2, 32'd1000000, 32'd0, 16'd21845);  // -43691 = -240deg
    wait_measure_done;

    if (fundamental_freq == 32'd50)
        $display("PASS: fundamental_freq=%0d Hz (bin1 x 50)", fundamental_freq);
    else
        $display("FAIL: fundamental_freq=%0d (expect 50)", fundamental_freq);

    $display("  rms A/B/C = %0d/%0d/%0d (0.1V)", rms_va, rms_vb, rms_vc);
    if (rms_va == rms_vb && rms_vb == rms_vc && rms_va > 0)
        $display("PASS: balanced RMS equal");
    else
        $display("FAIL: RMS unequal");

    if (phase_sequence == 1'b1)
        $display("PASS: phase_sequence=1 (ABC positive)");
    else
        $display("FAIL: phase_sequence=%b (expect 1)", phase_sequence);

    $display("  phase_ab=%0d phase_bc=%0d phase_ca=%0d (43691=+/-120deg)",
             phase_ab, phase_bc, phase_ca);
    // 激励 21845 本身量化误差 ±1（65536/3 非整数），容差 ±2
    if (phase_ab >= 16'd43689 && phase_ab <= 16'd43693 &&
        phase_bc >= 16'd43689 && phase_bc <= 16'd43693 &&
        phase_ca >= 16'd43689 && phase_ca <= 16'd43693)
        $display("PASS: 120deg phase differences");
    else
        $display("FAIL: phase outputs");

    if (unbalance_degree == 32'd0)
        $display("PASS: unbalance=0 (balanced)");
    else
        $display("FAIL: unbalance=%0d (expect 0)", unbalance_degree);

    if (thd_va == 32'd0)
        $display("PASS: pure sine THD=0");
    else
        $display("FAIL: THD=%0d (expect 0)", thd_va);

    $display("--- Test 3: Phase A with 20%% 3rd harmonic ---");
    send_round(3'd0, 32'd1000000, 32'd200000, 16'd0);
    wait_measure_done;
    send_round(3'd1, 32'd1000000, 32'd0, 16'd43691);
    wait_measure_done;
    send_round(3'd2, 32'd1000000, 32'd0, 16'd21845);
    wait_measure_done;

    $display("  thd_va=%0d (expect ~200)", thd_va);
    if (thd_va > 32'd180 && thd_va < 32'd220)
        $display("PASS: THD approx 200 permille");
    else
        $display("FAIL: THD=%0d (expect ~200)", thd_va);

    $display("  harm_pct_va: h1=%0d h3=%0d (expect h3~200)",
             harm_pct_va[15:0], harm_pct_va[47:32]);
    if (harm_pct_va[47:32] > 16'd180 && harm_pct_va[47:32] < 16'd220)
        $display("PASS: 3rd harmonic pct ~200 permille");
    else
        $display("FAIL: 3rd harmonic pct=%0d", harm_pct_va[47:32]);
    if (thd_vb == 32'd0 && thd_vc == 32'd0)
        $display("PASS: B/C clean");
    else
        $display("FAIL: B/C THD not zero");

    $display("--- Test 4: Unbalanced (A/B/C = 100%%/90%%/80%%) ---");
    send_round(3'd0, 32'd1000000, 32'd0, 16'd0);
    wait_measure_done;
    send_round(3'd1, 32'd900000, 32'd0, 16'd43691);
    wait_measure_done;
    send_round(3'd2, 32'd800000, 32'd0, 16'd21845);
    wait_measure_done;

    $display("  rms A/B/C = %0d/%0d/%0d (0.1V)", rms_va, rms_vb, rms_vc);
    $display("  unbalance=%0d permille (expect ~111)", unbalance_degree);
    if (unbalance_degree > 32'd95 && unbalance_degree < 32'd130)
        $display("PASS: unbalance approx 111 permille");
    else
        $display("CHECK: unbalance=%0d", unbalance_degree);

    $display("--- All measurement tests done ---");
    $finish;
end

initial begin
    #100_000_000;
    $display("TIMEOUT: measurement_tb");
    $finish;
end

endmodule
