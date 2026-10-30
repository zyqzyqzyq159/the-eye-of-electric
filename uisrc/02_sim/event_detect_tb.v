`timescale 1ns / 1ps
// event_detect ???????/??/???????
// ??? top.v ???sag=115/128(89.8%), swell=141/128(110.2%), int=13/128(10.2%)
// RMS ??? 0.1V?nominal 2200 = 220.0V?
module event_detect_tb;

reg         clk;
reg         rst_n;
reg  [31:0] rms_va;
reg  [31:0] rms_vb;
reg  [31:0] rms_vc;
reg         measure_done;
reg  [31:0] nominal_voltage;
reg  [7:0]  sag_threshold;
reg  [7:0]  swell_threshold;
reg  [7:0]  interrupt_threshold;
reg         enable;

wire [2:0]  event_alarm;
wire [31:0] event_count;

parameter PERIOD = 10;
always #(PERIOD/2) clk = ~clk;

// V26：64 条事件缓存已删（读出端口在 top 级悬空、无消费方），TB 只测
// 检测/确认/报警/计数这些保留的可观察行为
event_detect #(
    .RMS_BITS    (32)
) uut (
    .clk                (clk),
    .rst_n              (rst_n),
    .rms_va             (rms_va),
    .rms_vb             (rms_vb),
    .rms_vc             (rms_vc),
    .measure_done       (measure_done),
    .nominal_voltage    (nominal_voltage),
    .sag_threshold      (sag_threshold),
    .swell_threshold    (swell_threshold),
    .interrupt_threshold(interrupt_threshold),
    .event_alarm        (event_alarm),
    .event_count        (event_count),
    .enable             (enable)
);

task send_measure_pulse;
    begin
        @(posedge clk);
        measure_done = 1'b1;
        @(posedge clk);
        measure_done = 1'b0;
    end
endtask

integer i;

initial begin
    clk                 = 1'b0;
    rst_n               = 1'b0;
    rms_va              = 32'd2200;
    rms_vb              = 32'd2200;
    rms_vc              = 32'd2200;
    measure_done        = 1'b0;
    nominal_voltage     = 32'd2200;
    sag_threshold       = 8'd115;    // 89.8%??????
    swell_threshold     = 8'd141;    // 110.2%??????
    interrupt_threshold = 8'd13;     // 10.2%??????
    enable              = 1'b0;

    #1000;
    rst_n  = 1'b1;
    enable = 1'b1;
    #10000;

    $display("--- Test 1: Normal operation (no events) ---");
    rms_va = 32'd2200;
    rms_vb = 32'd2200;
    rms_vc = 32'd2200;
    for (i = 0; i < 60; i = i + 1) begin
        send_measure_pulse;
        #(PERIOD * 100);
    end
    $display("Normal: event_alarm=%b event_count=%0d (expect 0)", event_alarm, event_count);
    if (event_alarm == 3'b000 && event_count == 32'd0)
        $display("PASS: no false events");
    else
        $display("FAIL: false alarm during normal operation");

    $display("--- Test 2: Voltage sag on phase A (rms=180.0V < 89.8%% nominal) ---");
    rms_va = 32'd1800;
    for (i = 0; i < 15; i = i + 1) begin
        send_measure_pulse;
        #(PERIOD * 100);
    end
    $display("Sag A: event_alarm=%b (bit0=sag) event_count=%0d", event_alarm, event_count);
    if (event_alarm[0])
        $display("PASS: Sag detected on phase A");
    else
        $display("FAIL: Sag not detected");

    $display("--- Test 3: Voltage swell on phase B (rms=260.0V > 110.2%%) ---");
    rms_va = 32'd2200;
    rms_vb = 32'd2600;
    for (i = 0; i < 15; i = i + 1) begin
        send_measure_pulse;
        #(PERIOD * 100);
    end
    $display("Swell B: event_alarm=%b (bit1=swell) event_count=%0d", event_alarm, event_count);
    if (event_alarm[1])
        $display("PASS: Swell detected on phase B");
    else
        $display("FAIL: Swell not detected");

    $display("--- Test 4: Voltage interrupt on phase C (rms=100 = 10.0V) ---");
    rms_vb = 32'd2200;
    rms_vc = 32'd100;
    for (i = 0; i < 15; i = i + 1) begin
        send_measure_pulse;
        #(PERIOD * 100);
    end
    $display("Interrupt C: event_alarm=%b (bit2=interrupt) event_count=%0d", event_alarm, event_count);
    if (event_alarm[2])
        $display("PASS: Interrupt detected on phase C");
    else
        $display("FAIL: Interrupt not detected");

    $display("--- Test 5: Recovery (all normal) ---");
    rms_va = 32'd2200;
    rms_vb = 32'd2200;
    rms_vc = 32'd2200;
    for (i = 0; i < 20; i = i + 1) begin
        send_measure_pulse;
        #(PERIOD * 100);
    end
    $display("Recovery: event_alarm=%b (expect 0 after recovery)", event_alarm);
    if (event_alarm == 3'b000)
        $display("PASS: alarms cleared after recovery");
    else
        $display("FAIL: alarms not cleared");

    $display("--- All event_detect tests done ---");
    $finish;
end

initial begin
    #200_000_000;
    $display("TIMEOUT: event_detect_tb");
    $finish;
end

endmodule
