`timescale 1ns / 1ps
// fft_top ???256 ? FFT?????/?????mag_valid + bin_index?
// ????sample_sync ????? -> 256 ? valid_in ?? -> fft_done
// ???128 ? bin ???? mag_valid ???fft_done ????? bin ? 1 ?
module fft_top_tb;

reg         clk;
reg         rst_n;
reg  [11:0] data_in;
reg  [2:0]  channel_in;
reg         valid_in;
reg         sample_sync;
reg         enable;

wire [31:0] mag_out;
wire [15:0] phase_out;
wire [7:0]  bin_index;
wire        fft_done;
wire [2:0]  fft_channel;
wire        mag_valid;

parameter PERIOD = 10;
always #(PERIOD/2) clk = ~clk;

fft_top #(
    .FFT_SIZE  (256),
    .DATA_BITS (12)
) uut (
    .clk         (clk),
    .rst_n       (rst_n),
    .data_in     (data_in),
    .channel_in  (channel_in),
    .valid_in    (valid_in),
    .sample_sync (sample_sync),
    .mag_out     (mag_out),
    .phase_out   (phase_out),
    .bin_index   (bin_index),
    .fft_done    (fft_done),
    .fft_channel (fft_channel),
    .mag_valid   (mag_valid),
    .enable      (enable)
);

// ?????? bin ???/??
reg [31:0] mag_arr  [0:127];
reg [15:0] phase_arr [0:127];
integer    mag_cnt;

always @(posedge clk) begin
    if (mag_valid) begin
        mag_arr[bin_index]  <= mag_out;
        phase_arr[bin_index] <= phase_out;
        mag_cnt = mag_cnt + 1;
    end
end

real pi_val;
integer li;
reg [31:0] max_mag_val;
integer     max_mag_bin;

task feed_window(input integer fund_bin, input [11:0] amp, input [2:0] ch,
                 input integer harm3_en);
    integer n;
    real    angle, v;
    begin
        @(posedge clk);
        sample_sync = 1'b1;
        channel_in  = ch;
        @(posedge clk);
        sample_sync = 1'b0;
        for (n = 0; n < 256; n = n + 1) begin
            angle = 2.0 * pi_val * n * fund_bin / 256.0;
            v = 2048.0 + (amp - 2048.0) * $sin(angle);
            if (harm3_en)
                v = v + (amp - 2048.0) * 0.2 * $sin(3.0 * angle);
            @(posedge clk);
            data_in  = $rtoi(v);
            valid_in = 1'b1;
        end
        @(posedge clk);
        valid_in = 1'b0;
    end
endtask

task wait_fft_done;
    integer w;
    begin
        w = 0;
        while (!fft_done && w < 2_000_000) begin
            @(posedge clk);
            w = w + 1;
        end
        @(posedge clk);   // ? bin127 ? mag_valid ??
    end
endtask

task find_peak(input integer start_bin);
    integer i;
    begin
        max_mag_val = 32'd0;
        max_mag_bin = start_bin;
        for (i = start_bin; i < 128; i = i + 1) begin
            if (mag_arr[i] > max_mag_val) begin
                max_mag_val = mag_arr[i];
                max_mag_bin = i;
            end
        end
        if (start_bin == 1)
            $display("  bin1=%0d bin2=%0d bin6=%0d", mag_arr[1], mag_arr[2], mag_arr[6]);
    end
endtask

task dump_all_bins;
    integer d;
    begin
        for (d = 0; d < 128; d = d + 1)
            $display("  bin[%0d] = %0d", d, mag_arr[d]);
    end
endtask

integer     wait_cnt;

initial begin
    clk        = 1'b0;
    rst_n      = 1'b0;
    data_in    = 12'd0;
    channel_in = 3'd0;
    valid_in   = 1'b0;
    sample_sync= 1'b0;
    enable     = 1'b0;
    mag_cnt    = 0;
    pi_val     = 3.14159265;
    for (li = 0; li < 128; li = li + 1) begin
        mag_arr[li] = 32'd0;
        phase_arr[li] = 16'd0;
    end

    #1000;
    rst_n  = 1'b1;
    enable = 1'b1;
    #10000;

    $display("--- Test 1: Pure sine at bin 2 ---");
    feed_window(2, 12'd3848, 3'd0, 0);
    wait_fft_done;
    dump_all_bins;
    if (mag_cnt == 128)
        $display("PASS: 128 bins streamed");
    else
        $display("FAIL: bin count = %0d (expect 128)", mag_cnt);
    find_peak(1);
    if (max_mag_bin == 2)
        $display("PASS: peak at bin %0d (expect 2)", max_mag_bin);
    else
        $display("FAIL: peak at bin %0d (expect 2)", max_mag_bin);
    $display("  phase at bin2 = %0d (65536=360deg)", phase_arr[2]);
    if (phase_arr[2] > 16'd49052 && phase_arr[2] < 16'd49252)
        $display("PASS: sin phase ~ -90deg (49152)");
    else
        $display("FAIL: phase expect ~49152 got %0d", phase_arr[2]);
    if (fft_channel == 3'd0)
        $display("PASS: fft_channel=0");
    else
        $display("FAIL: fft_channel=%0d (expect 0)", fft_channel);

    #1000;

    $display("--- Test 2: Fundamental bin2 + 3rd harmonic bin6 (20%%) ---");
    mag_cnt = 0;
    feed_window(2, 12'd3848, 3'd0, 1);
    wait_fft_done;
    find_peak(1);
    if (max_mag_bin == 2)
        $display("PASS: fundamental peak at bin 2, mag=%0d", max_mag_val);
    else
        $display("FAIL: fundamental peak at bin %0d", max_mag_bin);
    if (mag_arr[6] > max_mag_val / 10 && mag_arr[6] < max_mag_val / 4)
        $display("PASS: 3rd harmonic mag=%0d (~20%% of %0d)", mag_arr[6], max_mag_val);
    else
        $display("CHECK: 3rd harmonic mag=%0d vs fundamental %0d", mag_arr[6], max_mag_val);

    #1000;

    $display("--- Test 3: DC input (peak at bin 0) ---");
    mag_cnt = 0;
    @(posedge clk);
    sample_sync = 1'b1;
    @(posedge clk);
    sample_sync = 1'b0;
    for (li = 0; li < 256; li = li + 1) begin
        @(posedge clk);
        data_in  = 12'd3000;
        valid_in = 1'b1;
    end
    @(posedge clk);
    valid_in = 1'b0;
    wait_fft_done;
    find_peak(0);
    if (max_mag_bin == 0)
        $display("PASS: DC peak at bin 0, mag=%0d", max_mag_val);
    else
        $display("FAIL: DC peak at bin %0d (expect 0)", max_mag_bin);
    if (mag_arr[1] < max_mag_val / 100)
        $display("PASS: bin1 leakage low (%0d)", mag_arr[1]);
    else
        $display("CHECK: bin1 = %0d", mag_arr[1]);

    #1000;

    $display("--- Test 4: Channel tracking ---");
    mag_cnt = 0;
    feed_window(2, 12'd3848, 3'd2, 0);
    wait_fft_done;
    if (fft_channel == 3'd2)
        $display("PASS: fft_channel=2");
    else
        $display("FAIL: fft_channel=%0d (expect 2)", fft_channel);

    $display("--- All fft_top tests done ---");
    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT: fft_top_tb");
    $finish;
end

endmodule
