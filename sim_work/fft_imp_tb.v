`timescale 1ns / 1ps
// 冲激测试：x[0]=4095 其余 0，正确 DFT 时所有 |X[k]|=4095
module fft_imp_tb;

reg         clk, rst_n;
reg  [11:0] data_in;
reg  [2:0]  channel_in;
reg         valid_in, sample_sync, enable;

wire [31:0] mag_out;
wire [15:0] phase_out;
wire [7:0]  bin_index;
wire        fft_done, mag_valid;
wire [2:0]  fft_channel;

parameter PERIOD = 10;
always #(PERIOD/2) clk = ~clk;

fft_top #(.FFT_SIZE(256), .DATA_BITS(12)) uut (
    .clk(clk), .rst_n(rst_n),
    .data_in(data_in), .channel_in(channel_in), .valid_in(valid_in),
    .sample_sync(sample_sync),
    .mag_out(mag_out), .phase_out(phase_out), .bin_index(bin_index),
    .fft_done(fft_done), .fft_channel(fft_channel), .mag_valid(mag_valid),
    .enable(enable)
);

integer n;
reg [31:0] mag_arr [0:127];
reg [15:0] ph_arr [0:127];

always @(posedge clk) begin
    if (mag_valid) begin
        mag_arr[bin_index] <= mag_out;
        ph_arr[bin_index]  <= phase_out;
    end
end

integer d;

initial begin
    clk = 0; rst_n = 0; data_in = 0; channel_in = 0;
    valid_in = 0; sample_sync = 0; enable = 0;
    #1000; rst_n = 1; enable = 1;
    #1000;

    // 冲激：x[0]=4095
    @(posedge clk); sample_sync = 1;
    @(posedge clk); sample_sync = 0;
    for (n = 0; n < 256; n = n + 1) begin
        @(posedge clk);
        data_in  = (n == 0) ? 12'd4095 : 12'd0;
        valid_in = 1'b1;
    end
    @(posedge clk); valid_in = 1'b0;

    wait (fft_done);
    @(posedge clk);
    for (d = 0; d < 128; d = d + 1)
        $display("bin[%0d] mag=%0d phase=%0d", d, mag_arr[d], ph_arr[d]);
    $finish;
end

initial begin
    #10_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
