`timescale 1ns / 1ps
// 时基窗口回归测试：持续灌入 256 样本/周期的正弦，逐时基档（1/2/4/8 周期）
// 抓取 A 窗迹线，统计过中线（y=154）次数 = 窗内周期数，并检查列覆盖无断裂。
// 回归目标：旧实现 4/8 周期档读地址回绕，同一份 2 周期数据重复 2/4 遍，
// 窗内呈现截断+重复的错乱波形。判据：每档过中线次数 = 1/2/4/8。
module wf_win_tb;

reg clk = 1'b0;
always #6.734 clk = ~clk;                    // 74.25MHz 像素时钟

reg rst_n = 1'b0;
wire vid_hs, vid_vs, vid_de;
wire [7:0] rgb_r, rgb_g, rgb_b;

reg [31:0] rms_va = 32'd2200, rms_vb = 32'd2200, rms_vc = 32'd2200;  // → ±50V 最大挡
reg [31:0] rms_ia = 32'd2200, rms_ib = 32'd2200, rms_ic = 32'd2200;
reg [31:0] thd_va = 32'd0, thd_vb = 32'd0, thd_vc = 32'd0;
reg [31:0] thd_ia = 32'd0, thd_ib = 32'd0, thd_ic = 32'd0;
reg [31:0] fundamental_freq = 32'd500;
reg [239:0] harm_pct_va = 240'd0, harm_pct_vb = 240'd0, harm_pct_vc = 240'd0;
reg [239:0] harm_pct_ia = 240'd0, harm_pct_ib = 240'd0, harm_pct_ic = 240'd0;
reg [15:0] phase_ab = 16'd43691, phase_bc = 16'd43691, phase_ca = 16'd43691;
reg [15:0] phase_ia = 16'd0, phase_ib = 16'd0, phase_ic = 16'd0;
reg phase_sequence = 1'b1;
reg [31:0] unbalance_degree = 32'd0;
reg mode_t = 1'b0;
reg freeze = 1'b0;
reg [2:0] event_alarm = 3'd0;
reg wf_t = 1'b0;
reg done_t = 1'b0;
reg [11:0] wfa = 12'd2048, wfb = 12'd2048, wfc = 12'd2048;
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

task pulse_func(input [10:0] d);
    begin
        func_d = d;
        repeat (4) @(negedge clk);
        func_t = ~func_t;
        repeat (4) @(negedge clk);
    end
endtask

task pulse_view(input [44:0] d);
    begin
        view_d = d;
        repeat (4) @(negedge clk);
        view_t = ~view_t;
        repeat (4) @(negedge clk);
    end
endtask

// 连续样本流：每样本推进 1/256 周期；A 相 ±1370 码（≈±60px @±50V 挡）
integer k;
real ph;
always begin
    @(negedge clk);
    ph  = 6.283185307179586 * k / 256.0;
    wfa = 2048 + $rtoi(1370.0 * $sin(ph));
    wf_t = ~wf_t;
    k = k + 1;
    repeat (200) @(negedge clk);              // 采样间隔 201 拍 > 偏置 FSM 90 拍
end

// ---- 帧内 A 窗迹线捕获：fnd/ycc 逐帧记录，帧尾快照到 fnd2/ycc2 ----
reg [10:0] px = 11'd0, py = 11'd0;
reg [8:0]  frames = 9'd0;
reg        fnd  [0:1023];
reg [10:0] ycc  [0:1023];
reg        fnd2 [0:1023];
reg [10:0] ycc2 [0:1023];
integer ci;
always @(posedge clk) begin
    if (vid_de) begin
        if (py >= 11'd64 && py < 11'd244 && px >= 11'd216 && px < 11'd1240 &&
            rgb_r == 8'hFF && rgb_g == 8'h55 && rgb_b == 8'h55) begin
            fnd[px[9:0] - 10'd216] <= 1'b1;
            ycc[px[9:0] - 10'd216] <= py;
        end
        if (px == 11'd1279 && py == 11'd719) begin
            for (ci = 0; ci < 1024; ci = ci + 1) begin
                fnd2[ci] <= fnd[ci];
                ycc2[ci] <= ycc[ci];
                fnd[ci]  <= 1'b0;
            end
            frames <= frames + 9'd1;
        end
        if (px == 11'd1279) begin
            px <= 11'd0;
            if (py == 11'd719) py <= 11'd0;
            else py <= py + 11'd1;
        end else px <= px + 11'd1;
    end
end

// 分析：等 2 个帧边界（其间一整帧完整扫过 A 窗），数过中线次数与列缺失
integer errors, col, cross, miss;
reg snd, sset;
task check_win(input integer expect_cycles, input [8*8-1:0] name);
    integer f0;
    begin
        cross = 0; miss = 0;
        f0 = frames;
        wait (frames == f0 + 9'd2);
        @(negedge clk);                       // 让快照 NBA 落地
        // 覆盖检查：左刻度 x217..264 与字符牌 x300..348 属合法遮挡（96 列）
        for (col = 0; col < 1024; col = col + 1)
            if (!fnd2[col]) miss = miss + 1;
        if (miss > 104) begin
            $display("[FAIL] %0s: %0d/1024 columns missing trace", name, miss);
            errors = errors + 1;
        end
        // 施密特式下穿计数：状态"下半"在 y>156 置位并计数，"回上"在 y<152 复位；
        // 迟滞作用于锁存状态而非相邻列差（相邻列步进 <1px，逐列比较永不触发）；
        // 首列只初始化状态不计数（窗口首列恰在下半时不算一次穿越）
        sset = 1'b0; snd = 1'b0;
        for (col = 0; col < 1024; col = col + 1) begin
            if (fnd2[col]) begin
                if (!sset) begin
                    snd = (ycc2[col] > 11'd156);
                    sset = 1'b1;
                end else if (!snd) begin
                    if (ycc2[col] > 11'd156) begin
                        snd = 1; cross = cross + 1;
                    end
                end else begin
                    if (ycc2[col] < 11'd152) snd = 0;
                end
            end
        end
        if (cross == expect_cycles)
            $display("PASS: %0s window shows %0d cycles (miss=%0d)", name, cross, miss);
        else begin
            $display("FAIL: %0s window shows %0d cycles (expect %0d, miss=%0d)",
                     name, cross, expect_cycles, miss);
            errors = errors + 1;
        end
    end
endtask

// 等待 n 个样本（k 由样本进程自增）
task k_wait(input integer n);
    integer k0;
    begin
        k0 = k;
        wait (k - k0 >= n);
    end
endtask

initial begin
    k = 0; errors = 0;
    rst_n = 1'b0;
    #133.4;
    rst_n = 1'b1;
    #900000;                                   // 等静态文本 fmt 初始化

    // 锁存测量值（挡位判定 → ±50V）并切到 3P4W·仅电压（隐藏 I 迹线）
    @(negedge clk); done_t = ~done_t; repeat (4) @(negedge clk);
    done_t = ~done_t;        repeat (4) @(negedge clk);
    pulse_func({2'd2, 2'd0, 2'd1, 3'd0, 2'd0});

    // 默认 tb=0（2 周期窗）：灌满整环后校验
    k_wait(560);
    check_win(2, "tb=0");

    // 1 周期窗
    pulse_view({3'sb111, 9'd0, 8'd0, 8'd0, 8'd0, 3'd0, 3'd0, 3'd0});
    k_wait(560);
    check_win(1, "tb=-1");

    // 4 周期窗（写侧每 2 点取 1）
    pulse_view({3'sd1, 9'd0, 8'd0, 8'd0, 8'd0, 3'd0, 3'd0, 3'd0});
    k_wait(1120);
    check_win(4, "tb=+1");

    // 8 周期窗（写侧每 4 点取 1）
    pulse_view({3'sd2, 9'd0, 8'd0, 8'd0, 8'd0, 3'd0, 3'd0, 3'd0});
    k_wait(2240);
    check_win(8, "tb=+2");

    if (errors == 0) $display("=== wf_win_tb ALL PASS ===");
    else             $display("=== wf_win_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #400_000_000;                              // 400ms 超时
    $display("TIMEOUT: wf_win_tb (errors=%0d)", errors);
    $finish;
end

endmodule
