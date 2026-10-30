`timescale 1ns / 1ps
// 直流居中探针：代码块从 top.v 原文提取（脚本生成，保证与实现逐字一致）
// 激励：12.8kHz 采样脉冲 + 50Hz 正弦(±400 码)叠加直流偏置 +200，验证：
//   1) ch_dc 在 ~2 周期内收敛到 2048+200=2248（±2 码）
//   2) dc_center 输出波形以 2048 为中心（均值 ≈ 2048±2）
module dc_est_tb;
reg clk = 1'b0;
always #5 clk = ~clk;
reg rst_n = 1'b0;
reg samp = 1'b0;
integer k;
reg [11:0] x;
wire [71:0] ch_flat = {48'd0, x};
wire [71:0] dc_flat;

dc_probe dut(.clk_100m(clk), .rst_n(rst_n), .eff_sample_pulse(samp),
             .ch_flat(ch_flat), .dc_flat(dc_flat));
wire [11:0] ch_dc0 = dc_flat[11:0];
wire [11:0] cen = dut.dc_center(x, ch_dc0);

// 采样脉冲 12.8kHz（78µs 周期 = 7800 拍）
integer pc;
always @(posedge clk) begin
    if (pc >= 7799) begin pc <= 0; samp <= 1'b1; end
    else begin pc <= pc + 1; samp <= 1'b0; end
end
real ph;
always @(posedge clk) begin
    if (samp) begin
        ph = 6.283185307179586 * k / 256.0;
        x = 2248 + $rtoi(400.0 * $sin(ph));   // 均值 2248（+200 偏置）
        k = k + 1;
    end
end
integer errors, cyc;
initial begin
    k = 0; errors = 0; pc = 0; x = 12'd2248;
    #100; rst_n = 1'b1;
    // 等 4 个采样周期（80ms 仿真）：收敛应在前 1~2 周期内完成
    #80_000_000;
    if (ch_dc0 >= 2246 && ch_dc0 <= 2250)
        $display("PASS: ch_dc converged to %0d (expect 2248±2)", ch_dc0);
    else begin $display("FAIL: ch_dc = %0d", ch_dc0); errors = errors + 1; end
    // dc_center：一个完整周期再取均值应回到 2048±2
    begin : msum
        integer acc, n;
        acc = 0; n = 0;
        for (cyc = 0; cyc < 256; cyc = cyc + 1) begin
            @(posedge samp); #1;
            acc = acc + cen; n = n + 1;
        end
        acc = acc / n;
        if (acc >= 2046 && acc <= 2050)
            $display("PASS: centered mean = %0d (expect 2048±2)", acc);
        else begin $display("FAIL: centered mean = %0d", acc); errors = errors + 1; end
    end
    if (errors == 0) $display("=== dc_est_tb ALL PASS ===");
    else             $display("=== dc_est_tb %0d FAILURES ===", errors);
    $finish;
end
initial begin #200_000_000; $display("TIMEOUT"); $finish; end
endmodule

module dc_probe(input clk_100m, input rst_n, input eff_sample_pulse,
                input [71:0] ch_flat, output [71:0] dc_flat);
wire [11:0] ch_data [0:5];
assign ch_data[0] = ch_flat[11:0];  assign ch_data[1] = ch_flat[23:12];
assign ch_data[2] = ch_flat[35:24]; assign ch_data[3] = ch_flat[47:36];
assign ch_data[4] = ch_flat[59:48]; assign ch_data[5] = ch_flat[71:60];
// --- 显示路径直流居中：每通道按锁相采样窗（256 脉冲 = 1 工频周期）相干平均 ---
// 采样板 1.25V 偏置 ≠ 精确 REFIN/2，残余直流在细量程下把波形顶偏（上下不对称）；
// 锁相锁定时整周期箱平均纹波为零、20ms 即收敛；测量侧 RMS/THD 从 FFT bin1 起
// 累加天然无直流，PLL 过零阈值与 FFT 输入均不需减除，仅显示路径需要
reg [7:0]  wf_dc_cnt;
reg [19:0] wf_dc_acc [0:5];
reg [11:0] ch_dc    [0:5];
integer wi2;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        wf_dc_cnt <= 8'd0;
        for (wi2 = 0; wi2 < 6; wi2 = wi2 + 1) begin
            wf_dc_acc[wi2] <= 20'd0;
            ch_dc[wi2]     <= 12'd2048;
        end
    end else if (eff_sample_pulse) begin
        if (wf_dc_cnt == 8'd255) begin
            wf_dc_cnt <= 8'd0;
            for (wi2 = 0; wi2 < 6; wi2 = wi2 + 1) begin
                ch_dc[wi2]     <= wf_dc_acc[wi2][19:8];   // 刚结束周期的均值（>>8 = /256）
                wf_dc_acc[wi2] <= {8'd0, ch_data[wi2]};   // 新周期首样本
            end
        end else begin
            wf_dc_cnt <= wf_dc_cnt + 8'd1;
            for (wi2 = 0; wi2 < 6; wi2 = wi2 + 1)
                wf_dc_acc[wi2] <= wf_dc_acc[wi2] + {8'd0, ch_data[wi2]};
        end
    end
end

// 减直流并夹取回 0..4095（中心 2048）；自检源无直流直通
function [11:0] dc_center;
    input [11:0] x;
    input [11:0] d;
    reg signed [13:0] t;
    begin
        t = $signed({1'b0, x}) - $signed({1'b0, d}) + 14'sd2048;
        dc_center = (t < 14'sd0) ? 12'd0 : (t > 14'sd4095) ? 12'd4095 : t[11:0];
    end
endfunction

assign dc_flat = {5'd0, 24'd0, ch_dc[0]};
endmodule
