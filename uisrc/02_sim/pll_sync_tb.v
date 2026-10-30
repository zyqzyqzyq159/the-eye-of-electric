`timescale 1ns / 1ps
// pll_sync 全数字锁相环行为仿真
// 激励按真实 ADC 通道速率（1273 clk/点 ≈ 78.5kHz）送 A 相正弦：
//  1) 50Hz 锁定 + freq_out 校验
//  2) 采样脉冲密度 = 256 点/周期
//  3) 49Hz 频率阶跃跟踪
//  4) 51Hz 频率阶跃跟踪
module pll_sync_tb;

reg         clk;
reg         rst_n;
reg  [11:0] adc_data;
reg  [2:0]  adc_channel;
reg         adc_valid;
reg         enable;

wire        sample_pulse;
wire [31:0] freq_out;
wire [15:0] phase_out;
wire        locked;

parameter PERIOD = 10;
always #(PERIOD/2) clk = ~clk;

pll_sync #(
    .CLK_FREQ         (100_000_000),
    .SAMPLES_PER_CYCLE(256),
    .ADC_BITS         (12)
) uut (
    .clk         (clk),
    .rst_n       (rst_n),
    .adc_data    (adc_data),
    .adc_channel (adc_channel),
    .adc_valid   (adc_valid),
    .sample_pulse(sample_pulse),
    .freq_out    (freq_out),
    .phase_out   (phase_out),
    .locked      (locked),
    .enable      (enable)
);

// --- 信号源：正弦 LUT，每 1273 个时钟采一点（78.5kHz，贴近真实 ADC）---
real    pi_val;
reg [11:0] sine_table [0:255];
integer si;
initial begin
    pi_val = 3.14159265;
    for (si = 0; si < 256; si = si + 1)
        sine_table[si] = 12'd2048 + $rtoi(1800.0 * $sin(2.0 * pi_val * si / 256.0));
end

reg [63:0] phase_incr_50, phase_incr_49, phase_incr_51;
initial begin
    phase_incr_50 = 64'd4294967296 * 50 * 1273 / 100_000_000;
    phase_incr_49 = 64'd4294967296 * 49 * 1273 / 100_000_000;
    phase_incr_51 = 64'd4294967296 * 51 * 1273 / 100_000_000;
end

reg [31:0] phase_acc;
reg [31:0] cur_incr;
reg        gen_en;
reg [15:0] samp_div;

always @(posedge clk) begin
    if (samp_div >= 16'd1272) samp_div <= 16'd0;
    else samp_div <= samp_div + 16'd1;
end
wire samp_tick = (samp_div == 16'd1272);

always @(posedge clk) begin
    adc_valid <= 1'b0;
    if (gen_en && samp_tick) begin
        adc_valid   <= 1'b1;
        adc_data    <= sine_table[phase_acc[31:24]];
        adc_channel <= 3'd0;
    end
end

always @(posedge clk) begin
    if (gen_en && samp_tick)
        phase_acc <= phase_acc + cur_incr;
end

integer pulse_count;

initial begin
    clk         = 1'b0;
    rst_n       = 1'b0;
    enable      = 1'b0;
    adc_data    = 12'd2048;
    adc_channel = 3'd0;
    adc_valid   = 1'b0;
    gen_en      = 1'b0;
    cur_incr    = 32'd0;
    phase_acc   = 32'd0;
    samp_div    = 16'd0;
    pulse_count = 0;

    #1000;
    rst_n  = 1'b1;
    enable = 1'b1;
    #10000;

    $display("--- Test 1: Lock to 50Hz ---");
    cur_incr = phase_incr_50[31:0];
    gen_en   = 1'b1;
    #(240_000_000);
    $display("locked=%b freq_out=%0d (0.1Hz unit)", locked, freq_out);
    if (locked)
        $display("PASS: locked at 50Hz");
    else
        $display("FAIL: not locked after 240ms");
    if (freq_out >= 32'd490 && freq_out <= 32'd510)
        $display("PASS: freq_out ~50Hz (%0d)", freq_out);
    else
        $display("FAIL: freq_out=%0d (expect ~500)", freq_out);

    $display("--- Test 2: Sample pulse density (2 cycles @50Hz) ---");
    pulse_count = 0;
    repeat (4_000_000) begin
        @(posedge clk);
        if (sample_pulse) pulse_count = pulse_count + 1;
    end
    $display("pulses in 40ms: %0d (expect ~512 = 2 cycles x 256)", pulse_count);
    if (pulse_count >= 508 && pulse_count <= 516)
        $display("PASS: 256 samples per cycle");
    else
        $display("FAIL: pulse density wrong");

    $display("--- Test 3: Frequency step to 49Hz ---");
    cur_incr = phase_incr_49[31:0];
    #(400_000_000);
    $display("after 49Hz: locked=%b freq_out=%0d", locked, freq_out);
    if (locked && freq_out >= 32'd485 && freq_out <= 32'd495)
        $display("PASS: tracking 49Hz");
    else
        $display("FAIL: 49Hz tracking");

    $display("--- Test 4: Frequency step to 51Hz ---");
    cur_incr = phase_incr_51[31:0];
    #(400_000_000);
    $display("after 51Hz: locked=%b freq_out=%0d", locked, freq_out);
    if (locked && freq_out >= 32'd503 && freq_out <= 32'd517)
        $display("PASS: tracking 51Hz");
    else
        $display("FAIL: 51Hz tracking");

    $display("--- All pll_sync tests done ---");
    $finish;
end

initial begin
    #1_600_000_000;
    $display("TIMEOUT: pll_sync_tb");
    $finish;
end

endmodule
