`timescale 1ns / 1ps
// ws2812 行为仿真：位时序（0 码 0.4µs / 1 码 0.8µs ± 容差）、帧节拍、
// 操作事件亮脉冲（flash 置起）、报警路径无 X
module ws2812_tb;

reg clk = 1'b0;
always #5 clk = ~clk;                     // 100MHz

reg rst_n;
reg [1:0] func_mode, disp_mode, meas_chan, meas_pick;
reg freeze, meas_vi;
reg [2:0] self_sel, event_alarm;
reg [9:0] ui_event;
wire din;

ws2812 #(
    .NLED(10), .CLK_HZ(100_000_000)
) uut (
    .clk(clk), .rst_n(rst_n),
    .func_mode(func_mode), .disp_mode(disp_mode), .freeze(freeze),
    .self_sel(self_sel), .meas_chan(meas_chan),
    .meas_vi(meas_vi), .meas_pick(meas_pick),
    .ui_event(ui_event), .event_alarm(event_alarm),
    .din(din)
);

// ---- 位宽监测：din 下降沿时归类当前高电平宽度 ----
reg din_d;
integer hw;
integer n0, n1, nbad;
always @(posedge clk) begin
    din_d <= din;
    if (din) hw = hw + 1;
    if (din_d && !din) begin              // 下降沿
        if      (hw >= 25 && hw <= 55) n0 = n0 + 1;   // 0 码 0.4µs±150ns
        else if (hw >= 65 && hw <= 95) n1 = n1 + 1;   // 1 码 0.8µs±150ns
        else                           nbad = nbad + 1;
    end
    if (!din && !din_d) hw = 0;
end

integer frames;
always @(posedge clk) if (uut.frame_load) frames = frames + 1;

integer errors;
initial begin
    rst_n = 1'b0;
    func_mode = 2'd0; disp_mode = 2'd0; freeze = 1'b0;
    self_sel = 3'd0; meas_chan = 2'd0; meas_vi = 1'b0; meas_pick = 2'd0;
    event_alarm = 3'd0; ui_event = 10'd0;
    hw = 0; n0 = 0; n1 = 0; nbad = 0; frames = 0; errors = 0;

    #133;
    rst_n = 1'b1;

    // --- Test 1: 3ms 基础流（开机彩虹期）位时序 ---
    #3_000_000;
    $display("pulses: zero=%0d one=%0d bad=%0d frames=%0d", n0, n1, nbad, frames);
    if (nbad == 0 && n0 > 100 && n1 > 100)
        $display("PASS: bit timing within 0.4/0.8us +/-150ns");
    else begin $display("FAIL: bit timing bad=%0d n0=%0d n1=%0d", nbad, n0, n1); errors=errors+1; end
    if (frames >= 3 && frames <= 4)
        $display("PASS: frame cadence ~0.8ms (3ms -> %0d frames)", frames);
    else begin $display("FAIL: frames=%0d (expect 3..4)", frames); errors=errors+1; end

    // --- Test 2: 按键事件 → flash 置起 ---
    @(negedge clk); ui_event[3] = 1'b1; @(negedge clk); ui_event[3] = 1'b0;
    #1_000_000;                            // 等下一帧装载消费 sticky
    if (uut.flash[3] >= 8'd250)
        $display("PASS: SEL1 key event sets flash (=%0d)", uut.flash[3]);
    else begin $display("FAIL: flash[3]=%0d", uut.flash[3]); errors=errors+1; end
    if (uut.flash[0] == 8'd0)
        $display("PASS: other LEDs not flashed");
    else begin $display("FAIL: flash[0]=%0d", uut.flash[0]); errors=errors+1; end

    // --- Test 3: 编码器事件 + 报警，流仍正常无 X ---
    @(negedge clk); ui_event[9] = 1'b1; @(negedge clk); ui_event[9] = 1'b0;
    #500_000;
    event_alarm = 3'b001;
    #2_000_000;
    if (nbad == 0 && ^din !== 1'bx)
        $display("PASS: alarm mode bit stream still clean (bad=%0d)", nbad);
    else begin $display("FAIL: alarm stream bad=%0d din=%b", nbad, din); errors=errors+1; end
    if (uut.flash[9] != 8'd0)
        $display("PASS: ENC4 event flashed");
    else begin $display("FAIL: flash[9]=0"); errors=errors+1; end
    event_alarm = 3'd0;

    // --- Test 4: 灯效颜色非零（彩虹期三通道有输出）---
    if (|uut.col_reg[0] && |uut.col_reg[9])
        $display("PASS: sweep colors active");
    else begin $display("FAIL: sweep colors zero"); errors=errors+1; end

    if (errors == 0)
        $display("=== ws2812_tb ALL PASS ===");
    else
        $display("=== ws2812_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #10_000_000;
    $display("TIMEOUT: ws2812_tb");
    $finish;
end

endmodule
