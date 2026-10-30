`timescale 1ns / 1ps
// 每窗量程偏移应用探针：display_ctrl 直驱，常数偏置信号检查 off_ram 三窗 V 偏移
// rms=77 → 自动挡 idx4（m=1028,s=13）；常值 code=2348（+300 偏置）
//   soff=0：三窗 V off = 300×1028>>13 = 37px
//   soff=+1：三窗 off = 300×720>>13 = 26px
//   仅 B 窗 +2：Vb off = 300×1029>>14 = 18px，Va/Vc 仍 26px
module soff_win_tb;

reg clk = 1'b0;
always #6.734 clk = ~clk;
reg rst_n = 1'b0;
wire vid_hs, vid_vs, vid_de;
wire [7:0] rgb_r, rgb_g, rgb_b;

reg [31:0] rms_va = 32'd77, rms_vb = 32'd77, rms_vc = 32'd77;
reg [31:0] rms_ia = 32'd77, rms_ib = 32'd77, rms_ic = 32'd77;
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
reg done_t = 1'b0, wf_t = 1'b0;
reg [11:0] wfa = 12'd2348, wfb = 12'd2348, wfc = 12'd2348;
reg [11:0] wfi_a = 12'd2048, wfi_b = 12'd2048, wfi_c = 12'd2048;
reg func_t = 1'b0;
reg [10:0] func_d = 11'd0;
reg view_t = 1'b0;
reg [44:0] view_d = 45'd0;
reg [177:0] pwr_bus_d = 178'd0;

display_ctrl #(
    .H_RES(1280), .V_RES(720), .DATA_BITS(12), .HARMONICS_NUM(15)
) dut (
    .clk(clk), .rst_n(rst_n),
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

integer k;
always begin
    @(negedge clk);
    wf_t = ~wf_t;
    repeat (200) @(negedge clk);
end

task fill(input integer n);
    integer j;
    begin
        for (j = 0; j < n; j = j + 1) @(wf_t);
    end
endtask

integer errors;

initial begin
    errors = 0;
    #133.4;
    rst_n = 1'b1;
    #900000;
    @(negedge clk); done_t = ~done_t; repeat (4) @(negedge clk);
    done_t = ~done_t;        repeat (4) @(negedge clk);

    // 1) 自动挡（idx4）：三窗 V 偏移应 = 37
    fill(560);
    check(37, 37, 37, "auto  idx4");

    // 2) 三窗 +1（idx5）：26/26/26
    view_d = {3'sd0, 9'd0, 8'd0, 8'd0, 8'd0, 3'sd1, 3'sd1, 3'sd1};
    @(negedge clk); view_t = ~view_t;
    fill(560);
    check(26, 26, 26, "soff+1 idx5");

    // 3) 仅 B 窗 +2（A/C 保持 +1）：26/18/26
    view_d = {3'sd0, 9'd0, 8'd0, 8'd0, 8'd0, 3'sd1, 3'sd2, 3'sd1};
    @(negedge clk); view_t = ~view_t;
    fill(560);
    check(26, 18, 26, "B only +2");

    // 4) 仅 B 窗 -4（饱和到 idx0，m=7200,s=13）：Va 26 / Vb 钳位 92 / Vc 26
    view_d = {3'sd0, 9'd0, 8'd0, 8'd0, 8'd0, 3'sd1, -3'sd4, 3'sd1};
    @(negedge clk); view_t = ~view_t;
    fill(560);
    check(26, 92, 26, "B -4 saturate");

    if (errors == 0) $display("=== soff_win_tb ALL PASS ===");
    else             $display("=== soff_win_tb %0d FAILURES ===", errors);
    $finish;
end

task check(input integer ea, eb, ec, input [8*16-1:0] name);
    integer a0, b0, c0;
    begin
        // 读回 4 个地址（环形数据一致）取窗 V 偏移字节
        a0 = dut.off_ram[10][7:0];   b0 = dut.off_ram[10][15:8];   c0 = dut.off_ram[10][23:16];
        $display("%0s: off = %0d/%0d/%0d (expect %0d/%0d/%0d)%0s",
                 name, a0, b0, c0, ea, eb, ec,
                 (a0==ea && b0==eb && c0==ec) ? "  PASS" : "  FAIL");
        if (!(a0==ea && b0==eb && c0==ec)) errors = errors + 1;
    end
endtask

initial begin
    #80_000_000;
    $display("TIMEOUT soff_win_tb");
    $finish;
end

endmodule
