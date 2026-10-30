`timescale 1ns / 1ps
// UI 目检 TB：渲染各功能/视图组合的整帧 PPM，由 ui_ppm2png.py 转 PNG 目检
// 覆盖：V/I 双显（相色 V + 琥珀 I 叠加）、V/I 三态切换（SEL2）、电流频谱/谐波表、
//       通道循环选择（SEL1）、时基/平移/垂直位置/缩放（编码器）、定格 STOP
// 波形刻意对齐网格方便目检：V 峰 1370 码、I 峰 1000 码（30° 滞后，感性负载）
// B 相注入 20% 三次 + 10% 五次谐波
module ui_frame_tb;

reg clk = 1'b0;
always #6.734 clk = ~clk;                    // 74.25MHz 像素时钟

reg rst_n = 1'b0;
wire vid_hs, vid_vs, vid_de;
wire [7:0] rgb_r, rgb_g, rgb_b;

reg [31:0] rms_va = 32'd2200, rms_vb = 32'd2135, rms_vc = 32'd2264;  // 0.1V
reg [31:0] thd_va = 32'd123,   thd_vb = 32'd48,   thd_vc = 32'd31;    // ‰（3 位数验证 cfg_base=8 修复）
reg [31:0] rms_ia = 32'd85,    rms_ib = 32'd82,   rms_ic = 32'd88;    // 0.1A
reg [31:0] thd_ia = 32'd35,    thd_ib = 32'd52,   thd_ic = 32'd41;    // ‰
reg [31:0] fundamental_freq = 32'd500;                               // 0.1Hz = 50.0Hz
reg [239:0] harm_pct_va = 240'd0, harm_pct_vb = 240'd0, harm_pct_vc = 240'd0;
reg [239:0] harm_pct_ia = 240'd0, harm_pct_ib = 240'd0, harm_pct_ic = 240'd0;
reg [15:0] phase_ab = 16'd43691, phase_bc = 16'd43691, phase_ca = 16'd43691;  // -120°
reg [15:0] phase_ia = -16'd5461, phase_ib = -16'd5461, phase_ic = -16'd5461;  // -30° 感性
reg phase_sequence = 1'b1;
reg [31:0] unbalance_degree = 32'd18;
reg mode_t = 1'b0;
reg freeze = 1'b0;
reg [2:0] event_alarm = 3'd0;

reg wf_t = 1'b0;
reg done_t = 1'b0;
reg [11:0] wfa = 12'd2048, wfb = 12'd2048, wfc = 12'd2048;
reg [11:0] wfi_a = 12'd2048, wfi_b = 12'd2048, wfi_c = 12'd2048;

reg func_t = 1'b0;
reg [10:0] func_d = 11'd0;                   // {func_mode[1:0], meas_chan[1:0], meas_vi[1:0], self_sel[2:0], meas_pick[1:0]}
reg view_t = 1'b0;
reg [44:0] view_d = 45'd0;
// pwr_bus = {pf_a Q15, pa,pb,pc16, qa,qb,qc16, ptot,qtot16, cap,ind16, cpres,pfok,lag}
// V=220.0V I=8.5A φ=-30° → PF=0.87 P≈162W Q≈94var，感性需补偿 C≈59µF
reg [177:0] pwr_bus_d = {15'sd28312, 16'd1620, 16'd1560, 16'd1680,
                         16'd940, 16'd900, 16'd980, 16'd4860, 16'd2820,
                         16'd59, 16'd0, 1'b1, 1'b0, 1'b1};

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
    .done_toggle(done_t),
    .wf_toggle(wf_t), .wf_va(wfa), .wf_vb(wfb), .wf_vc(wfc),
    .wf_ia(wfi_a), .wf_ib(wfi_b), .wf_ic(wfi_c),
    .mode_toggle(mode_t), .freeze(freeze), .event_alarm(event_alarm),
    .func_toggle(func_t), .func_data(func_d),
    .view_toggle(view_t), .view_data(view_d),
    .pwr_bus(pwr_bus_d),
    .enable(1'b1)
);

// 功能状态翻转（数据已稳定 2 拍后翻握手，与 user_input 行为一致）
task pulse_func(input [10:0] d);
    begin
        func_d = d;
        repeat (4) @(negedge clk);
        func_t = ~func_t;
        repeat (4) @(negedge clk);
    end
endtask

// 视图参数翻转：tb[42:40] s3 / pan[39:31] / vpos_a[29:22] / vpos_b[21:14] / vpos_c[13:6]
//               / gain_a[5:4] / gain_b[3:2] / gain_c[1:0]
task pulse_view(input [44:0] d);
    begin
        view_d = d;
        repeat (4) @(negedge clk);
        view_t = ~view_t;
        repeat (4) @(negedge clk);
    end
endtask

// 谐波示例数据：电压 A 含 3/5 次，电流 A 含 3/5 次（跟踪注入），电流 C 含 2/4 次
initial begin
    harm_pct_va[15:0]  = 16'd1000;   // A h1 100%
    harm_pct_va[47:32] = 16'd200;    // A h3 20%
    harm_pct_va[79:64] = 16'd80;     // A h5 8%
    harm_pct_vb[15:0]  = 16'd1000;
    harm_pct_vb[47:32] = 16'd120;
    harm_pct_vb[79:64] = 16'd50;
    harm_pct_vc[15:0]  = 16'd1000;
    harm_pct_vc[31:16] = 16'd60;
    harm_pct_vc[63:48] = 16'd30;
    harm_pct_vc[111:96]= 16'd40;
    harm_pct_ia[15:0]  = 16'd1000;
    harm_pct_ia[47:32] = 16'd200;
    harm_pct_ia[79:64] = 16'd100;
    harm_pct_ib[15:0]  = 16'd1000;
    harm_pct_ib[47:32] = 16'd150;
    harm_pct_ic[15:0]  = 16'd1000;
    harm_pct_ic[31:16] = 16'd70;
end

// ---- 帧捕获：按 vid_de 顺序写入帧缓冲 ----
reg cap = 1'b0;
reg [10:0] px = 11'd0, py = 11'd0;
reg [7:0] frames = 8'd0;
reg [23:0] fb [0:921599];

always @(posedge clk) begin
    if (vid_de) begin
        if (cap) fb[py * 1280 + px] <= {rgb_r, rgb_g, rgb_b};
        if (px == 11'd1279) begin
            px <= 11'd0;
            if (py == 11'd719) begin py <= 11'd0; frames <= frames + 8'd1; end
            else py <= py + 11'd1;
        end else px <= px + 11'd1;
    end
end

reg [8*24-1:0] dump_name;
task dump_ppm;
    integer f, i;
    begin
        f = $fopen(dump_name, "wb");
        $fwrite(f, "P6\n1280 720\n255\n");
        for (i = 0; i < 921600; i = i + 1)
            $fwrite(f, "%c%c%c", fb[i][23:16], fb[i][15:8], fb[i][7:0]);
        $fclose(f);
        $display("dumped %0s", dump_name);
    end
endtask

// 捕捉 1 帧（等 1 帧稳定后再采 1 帧）
task capture_one;
    begin
        wait (frames[0] == 1'b0);
        @(negedge clk);
        wait (frames[0] == 1'b1);
        cap = 1'b1;
        wait (frames[0] == 1'b0);
        cap = 1'b0;
    end
endtask

// SW2 模式切换：top.v 中 mode_toggle 是翻转信号（每按一次电平翻转一次并保持）
task toggle_mode;
    begin
        @(negedge clk);
        mode_t = ~mode_t;
        repeat (4) @(negedge clk);
    end
endtask


// 连续样本流：n 个样本（256 样本/周期，电流滞后电压 30°）；
// 采样间隔 201 周期 > 偏置预计算 FSM 的 90 周期（真实硬件为 586 周期）
integer k;
real ph, phb, phi;

task fill_wave(input integer n);
    integer j;
    begin
        for (j = 0; j < n; j = j + 1) begin
            ph  = 6.283185307179586 * k / 256.0;
            phb = ph - 2.0943951023931953;
            phi = ph - 0.5235987755982988;      // -30°
            @(negedge clk);
            wfa = 2048 + $rtoi(1370.0 * $sin(ph));
            wfb = 2048 + $rtoi(1370.0 * $sin(phb) + 274.0 * $sin(3.0 * phb)
                              + 137.0 * $sin(5.0 * phb));
            wfc = 2048 + $rtoi(685.0 * $sin(ph + 2.0943951023931953));
            wfi_a = 2048 + $rtoi(1000.0 * $sin(phi));
            wfi_b = 2048 + $rtoi(1000.0 * $sin(phi - 2.0943951023931953)
                               + 200.0 * $sin(3.0 * (phi - 2.0943951023931953)));
            wfi_c = 2048 + $rtoi(1000.0 * $sin(phi + 2.0943951023931953));
            wf_t = ~wf_t;
            k = k + 1;
            repeat (200) @(negedge clk);
        end
    end
endtask
initial begin
    k = 0;
    rst_n = 1'b0;
    #133.4;
    rst_n = 1'b1;
    // 等内部 fmt 初始化计数（65536 拍 ≈ 0.88ms）完成静态文本写入
    #900000;
    // 先触发一轮测量锁存：rms_px=2200 等 → 自适应挡位判定为最宽挡
    @(negedge clk);
    done_t = ~done_t;
    repeat (4) @(negedge clk);
    done_t = ~done_t;
    repeat (4) @(negedge clk);

    // 填充波形缓冲（初始 512 点 = 2 周期；切时基后按新抽点率补灌整环）
    fill_wave(512);
    @(negedge clk);
    done_t = ~done_t;                 // 锁存一轮测量值（面板数字上屏）
    @(negedge clk);
    done_t = ~done_t;

    // 1. 自检模式 · 波形 · V/I 双显（相色 V + 琥珀 I 叠加）
    capture_one;
    dump_name = "ui_m0_self.ppm"; dump_ppm;

    // 2. 三相四线 · 波形 · V/I 双显
    pulse_func({2'd2, 2'd0, 2'd0, 3'd0, 2'd0});
    capture_one;
    dump_name = "ui_m0_3p4w.ppm"; dump_ppm;

    // 3. 三相四线 · 波形 · 仅电压（SEL2，琥珀迹线隐藏）
    pulse_func({2'd2, 2'd0, 2'd1, 3'd0, 2'd0});
    capture_one;
    dump_name = "ui_m0_vonly.ppm"; dump_ppm;

    // 4. 三相四线 · 波形 · 仅电流（面板切 Ia/Ib/Ic + 单位 A + V-I 相位角）
    pulse_func({2'd2, 2'd0, 2'd2, 3'd0, 2'd0});
    capture_one;
    dump_name = "ui_m0_ionly.ppm"; dump_ppm;

    // 5. 电流频谱（vi_cur：频谱柱取电流谐波）
    toggle_mode;                      // 波形 → 频谱
    capture_one;
    dump_name = "ui_m1_spec_i.ppm"; dump_ppm;

    // 6. 电压频谱（回双显后频谱柱恢复相色电压）
    pulse_func({2'd2, 2'd0, 2'd0, 3'd0, 2'd0});
    capture_one;
    dump_name = "ui_m1_spec_v.ppm"; dump_ppm;

    // 7. 电流谐波表（表格列 Ia/Ib/Ic）
    toggle_mode;                      // 频谱 → 谐波表
    pulse_func({2'd2, 2'd0, 2'd2, 3'd0, 2'd0});
    capture_one;
    dump_name = "ui_m2_tab_i.ppm"; dump_ppm;

    // 8. 谐波表 → 波形，通道选择 = A（B/C 窗隐藏）
    toggle_mode;                      // 谐波表 → 波形
    pulse_func({2'd2, 2'd1, 2'd0, 3'd0, 2'd0});
    capture_one;
    dump_name = "ui_m0_chA.ppm"; dump_ppm;

    // 9. 编码器视图：时基 4 周期(tb=+1)、平移 64 样本、A 相上移 24px、B 相量程 -2 挡（rms 在最大挡 7，+2 会饱和，-2 才有缩放空间）
    //（先恢复全通道显示——用例 8 的 chA 模式会遮蔽 B/C 窗；切时基后按 1/2 抽点率补灌）
    pulse_func({2'd2, 2'd0, 2'd0, 3'd0, 2'd0});
    pulse_view({3'sd1, 9'd64, 8'hE8, 8'd0, 8'd0, 3'd0, -3'sd2, 3'd0});
    fill_wave(1120);
    capture_one;
    dump_name = "ui_view.ppm"; dump_ppm;

    // 9b. 时基 8 周期(tb=+2)：补灌 2048+ 样本后整窗 8 周期
    pulse_view({3'sd2, 9'd0, 8'd0, 8'd0, 8'd0, 3'd0, 3'd0, 3'd0});
    fill_wave(2240);
    capture_one;
    dump_name = "ui_tb8.ppm"; dump_ppm;

    // 9c. 时基 1 周期(tb=-1)：回满速率补灌 512+ 样本
    pulse_view({3'sb111, 9'd0, 8'd0, 8'd0, 8'd0, 3'd0, 3'd0, 3'd0});
    fill_wave(560);
    capture_one;
    dump_name = "ui_tb1.ppm"; dump_ppm;

    // 10. 恢复通道全显 + 默认视图，定格（STOP 红徽章）
    pulse_func({2'd2, 2'd0, 2'd0, 3'd0, 2'd0});
    pulse_view(45'd0);
    @(negedge clk);
    freeze = 1'b1;
    capture_one;
    dump_name = "ui_stop.ppm"; dump_ppm;
    freeze = 1'b0;

    $display("UI frames done");
    $finish;
end

endmodule
