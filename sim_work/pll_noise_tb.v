`timescale 1ns / 1ps
// pll_sync 噪声鲁棒性测试：50Hz 正弦（±300 码）叠加 ±24 码随机噪声，
// ch0 标记的有效脉冲间隔 8.8µs（复刻 ADC 6 通道轮询节拍）。
// 回归目标：旧过零检测无滞回，噪声在过零附近来回穿线一周期触发多次，
// PI 随机踢动 → 频率读数乱跳、锁定丢失。滞回版应稳定锁定且频率散布 <1Hz。
module pll_noise_tb;

reg clk = 1'b0;
always #5 clk = ~clk;
reg rst_n = 1'b0;

reg  [11:0] adc_data;
reg  [2:0]  adc_channel;
reg         adc_valid;

wire        sample_pulse;
wire [31:0] freq_out;
wire [15:0] phase_out;
wire        locked;

pll_sync #(
    .CLK_FREQ(100_000_000), .SAMPLES_PER_CYCLE(256), .ADC_BITS(12)
) dut (
    .clk(clk), .rst_n(rst_n),
    .adc_data(adc_data), .adc_channel(adc_channel), .adc_valid(adc_valid),
    .sample_pulse(sample_pulse), .freq_out(freq_out), .phase_out(phase_out),
    .locked(locked), .enable(1'b1)
);

// ADC 轮询节拍：每 883 拍一个 ch0 有效脉冲（≈8.8µs）
integer pc;
always @(posedge clk) begin
    if (pc >= 882) begin pc <= 0; adc_valid <= 1'b1; end
    else begin pc <= pc + 1; adc_valid <= 1'b0; end
end

// 信号：50Hz ±300 码 + ±24 码无偏噪声
real t, sig;
integer noise;
always @(posedge clk) if (pc == 400) begin
    t = $realtime;
    sig = 2048.0 + 300.0 * $sin(6.283185307179586 * t / 20_000_000.0);
    noise = ($random % 25);   // ±24 无偏噪声（%25 → −24..+24）
    adc_data = $rtoi(sig) + noise;
    adc_channel = 3'd0;
end

// 稳定性统计：锁定后 50ms 窗口内 freq_out 最大散布
integer fmax, fmin, fcur, samp_n;
reg mon_en;
always @(posedge clk) begin
    if (mon_en) begin
        fcur = freq_out;
        if (samp_n == 0) begin fmax = fcur; fmin = fcur; end
        else begin
            if (fcur > fmax) fmax = fcur;
            if (fcur < fmin) fmin = fcur;
        end
        samp_n = samp_n + 1;
    end
end

integer errors;
initial begin
    adc_data = 12'd2048; adc_channel = 3'd0; adc_valid = 1'b0;
    pc = 0; errors = 0; samp_n = 0; mon_en = 0; fmax = 0; fmin = 999999;
    #1000; rst_n = 1'b1;

    // 等锁定（250ms 预算）
    begin : wl
        integer w;
        for (w = 0; w < 250 && !locked; w = w + 1) #1_000_000;
    end
    if (locked) $display("PASS: locked at %0t", $time);
    else begin $display("FAIL: never locked"); errors = errors + 1; end

    // 锁定后监测 50ms 的频率散布
    #50_000_000; mon_en = 1'b1;
    #50_000_000; mon_en = 1'b0;
    $display("freq spread: %0d..%0d (0.1Hz units, span %0d)", fmin, fmax, fmax - fmin);
    if (fmax - fmin <= 10) $display("PASS: freq stable (<1.0 Hz span)");
    else begin $display("FAIL: freq span %0d > 10", fmax - fmin); errors = errors + 1; end
    if (fmin >= 490 && fmax <= 510) $display("PASS: freq tracks 50Hz");
    else begin $display("FAIL: freq out of 49-51Hz"); errors = errors + 1; end

    if (errors == 0) $display("=== pll_noise_tb ALL PASS ===");
    else             $display("=== pll_noise_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #500_000_000;
    $display("TIMEOUT pll_noise_tb (locked=%b)", locked);
    $finish;
end

endmodule
