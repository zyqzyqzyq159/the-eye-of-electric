`timescale 1ns / 1ps
// ENC4 端到端链路测试（手动量程版）：user_input(100MHz) ⇄ display_ctrl(74.25MHz) 按 top.v 接法互联，
// 真实四沿正交激励 ENC4 → 验证：
//   1) view_data 量程偏移字段 [8:6]/[5:3]/[2:0] 随旋转 0→+1→+2→0（三窗联动，pick=0）
//   2) 按压 ENC4_SW 三窗偏移清零（回自适应挡位）
//   3) 渲染迹线峰峰按挡位缩放：rms=77 → 自动 35V 挡（50px），+1 挡 → 25V 挡? 
//      msel_v: idx4(m=1028,s=13)=50px → idx5(m=720,s=13)=35px → idx6(m=1029,s=14)=25px
module enc4_chain_tb;

reg clk100 = 1'b0;
always #5 clk100 = ~clk100;                  // 100MHz 采样域
reg clk74 = 1'b0;
always #6.734 clk74 = ~clk74;                // 74.25MHz 像素域

// ---- user_input 激励 ----
reg rst_n = 1'b0;
reg btn_func=1, btn_view=1, btn_freeze=1, btn_sel1=1, btn_sel2=1, btn_sel3=1;
reg btn_sw2=0, btn_sw3=0;
reg enc1_a=1, enc1_b=1, enc2_a=1, enc2_b=1, enc3_a=1, enc3_b=1;
reg enc4_a=1, enc4_b=1;
reg enc1_sw=1, enc3_sw=1, enc4_sw=1;

wire [1:0]  func_mode, disp_mode;
wire        freeze;
wire [2:0]  self_sel;
wire [1:0]  meas_chan, meas_vi, meas_pick;
wire [10:0] func_data;
wire        func_toggle, mode_change_pulse;
wire [44:0] view_data;
wire        view_toggle;
wire [9:0]  ui_event;

user_input #(
    .CLK_FREQ(100_000_000), .DEBOUNCE_MS(1), .EXT_BTN_ACTIVE_LOW(1)
) uin (
    .clk(clk100), .rst_n(rst_n),
    .btn_func(btn_func), .btn_view(btn_view), .btn_freeze(btn_freeze),
    .btn_sel1(btn_sel1), .btn_sel2(btn_sel2), .btn_sel3(btn_sel3),
    .btn_sw2(btn_sw2), .btn_sw3(btn_sw3),
    .enc1_a(enc1_a), .enc1_b(enc1_b), .enc1_sw(enc1_sw),
    .enc2_a(enc2_a), .enc2_b(enc2_b),
    .enc3_a(enc3_a), .enc3_b(enc3_b), .enc3_sw(enc3_sw),
    .enc4_a(enc4_a), .enc4_b(enc4_b), .enc4_sw(enc4_sw),
    .func_mode(func_mode), .disp_mode(disp_mode), .freeze(freeze),
    .self_sel(self_sel), .meas_chan(meas_chan), .meas_vi(meas_vi), .meas_pick(meas_pick),
    .func_data(func_data), .func_toggle(func_toggle),
    .view_data(view_data), .view_toggle(view_toggle),
    .mode_change_pulse(mode_change_pulse), .ui_event(ui_event)
);

// ---- display_ctrl（除时钟域外与 top.v 相同的互联）----
wire vid_hs, vid_vs, vid_de;
wire [7:0] rgb_r, rgb_g, rgb_b;
reg [31:0] rms_va = 32'd77, rms_vb = 32'd77, rms_vc = 32'd77;  // 7.7V → 自动 35V 挡
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
reg frz = 1'b0;
reg [2:0] event_alarm = 3'd0;
reg done_t = 1'b0;
reg wf_t = 1'b0;                       // 先声明，避免例化处隐式 wire
reg [11:0] wfa = 12'd2048, wfb = 12'd2048, wfc = 12'd2048;
reg [11:0] wfi_a = 12'd2048, wfi_b = 12'd2048, wfi_c = 12'd2048;
reg [177:0] pwr_bus_d = 178'd0;

// 像素域复位链（复刻 top.v）
reg rst_meta, rst_px;
always @(posedge clk74 or negedge rst_n) begin
    if (!rst_n) begin rst_meta <= 1'b0; rst_px <= 1'b0; end
    else begin rst_meta <= 1'b1; rst_px <= rst_meta; end
end

display_ctrl #(
    .H_RES(1280), .V_RES(720), .DATA_BITS(12), .HARMONICS_NUM(15)
) disp (
    .clk(clk74), .rst_n(rst_px),
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
    .mode_toggle(mode_t), .freeze(frz), .event_alarm(event_alarm),
    .func_toggle(func_toggle), .func_data(func_data),
    .view_toggle(view_toggle), .view_data(view_data),
    .pwr_bus(pwr_bus_d), .enable(1'b1)
);

// 连续信号 ±400 码：35V 挡 ±50px → +1 挡 ±35px → +2 挡 ±25px（无削顶）
integer k;
real ph;
reg [1:0] wf_dly;
reg samp_pulse;
integer samp_cnt;
always @(posedge clk100) begin
    if (samp_cnt >= 200) begin samp_cnt <= 0; samp_pulse <= 1'b1; end
    else begin samp_cnt <= samp_cnt + 1; samp_pulse <= 1'b0; end
end
// 波形握手（复刻 top.v：数据先更新，2 拍后翻转）；单块内顺序执行无竞争
always @(posedge clk100 or negedge rst_n) begin
    if (!rst_n) begin wf_t <= 1'b0; wf_dly <= 2'd0; k = 0; end
    else begin
        wf_dly <= {wf_dly[0], samp_pulse};
        if (samp_pulse) begin
            ph = 6.283185307179586 * k / 256.0;
            wfa <= 2048 + $rtoi(400.0 * $sin(ph));
            k = k + 1;
        end
        if (wf_dly[1]) wf_t <= ~wf_t;
    end
end

// ---- ENC4 正交激励（一格 = 完整正交周期，CW = A 先落）----
task detent4(input cw);
    begin
        if (cw) begin enc4_a=0; #500; enc4_b=0; #500; enc4_a=1; #500; enc4_b=1; #500; end
        else    begin enc4_b=0; #500; enc4_a=0; #500; enc4_b=1; #500; enc4_a=1; #500; end
        #5000;
    end
endtask

// ---- view_toggle 沿捕获 view_data ----
reg [44:0] vd_cap;
reg        vd_new;
reg vt_d1;
always @(posedge clk100) begin
    vt_d1 <= view_toggle;
    if (vt_d1 && !view_toggle || !vt_d1 && view_toggle) begin
        vd_cap <= view_data;
        vd_new <= ~vd_new;
    end
end

// ---- A 窗迹线峰峰测量（帧快照）----
reg [10:0] px = 0, py = 0;
reg [8:0]  frames = 0;
reg        fnd [0:1023];
reg [10:0] ycc [0:1023];
reg        fnd2 [0:1023];
reg [10:0] ycc2 [0:1023];
integer ci;
always @(posedge clk74) begin
    if (vid_de) begin
        if (py >= 11'd64 && py < 11'd244 && px >= 11'd216 && px < 11'd1240 &&
            rgb_r == 8'hFF && rgb_g == 8'h55 && rgb_b == 8'h55) begin
            fnd[px[9:0] - 10'd216] <= 1'b1;
            ycc[px[9:0] - 10'd216] <= py;
        end
        if (px == 11'd1279 && py == 11'd719) begin
            for (ci = 0; ci < 1024; ci = ci + 1) begin
                fnd2[ci] <= fnd[ci]; ycc2[ci] <= ycc[ci]; fnd[ci] <= 1'b0;
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

function integer amp_pp;                      // A 窗 V 迹峰峰（px）
    input dummy;
    integer c2, mn, mx;
    begin
        mn = 2048; mx = 0;
        for (c2 = 0; c2 < 1024; c2 = c2 + 1)
            if (fnd2[c2]) begin
                if (ycc2[c2] < mn) mn = ycc2[c2];
                if (ycc2[c2] > mx) mx = ycc2[c2];
            end
        amp_pp = (mn == 2048) ? -1 : mx - mn;
    end
endfunction

task wait_frames(input integer n);
    integer f0;
    begin
        f0 = frames;
        wait (frames == f0 + n);
        @(negedge clk74);
    end
endtask

integer errors, a0, a1, a2, a3;

initial begin
    k = 0; errors = 0; samp_cnt = 0;
    rst_n = 1'b0;
    #133.4;
    rst_n = 1'b1;
    #900000;                                   // 像素域复位 + fmt 初始化

    @(negedge clk100); done_t = ~done_t; repeat (4) @(negedge clk74);
    done_t = ~done_t;        repeat (4) @(negedge clk74);   // 锁存挡位（±50V）

    // 基准：自动 35V 挡，迹线峰峰 ≈ 100px
    wait_frames(3);
    a0 = amp_pp(1'b0);
    if (a0 >= 94 && a0 <= 106) $display("PASS: auto-scale amplitude = %0d px", a0);
    else begin $display("FAIL: auto amplitude = %0d px (expect ~100)", a0); errors = errors + 1; end

    // 顺时针一格 → soff 0→+1（量程变粗 35V→50V 挡? msel idx5=±25V）：字段与幅度
    detent4(1);
    #10000;
    if ($signed(vd_cap[8:6]) == 1 && $signed(vd_cap[5:3]) == 1 && $signed(vd_cap[2:0]) == 1)
        $display("PASS: view_data soff -> +1 (all windows)");
    else begin $display("FAIL: soff = %0d/%0d/%0d", $signed(vd_cap[8:6]), $signed(vd_cap[5:3]), $signed(vd_cap[2:0])); errors = errors + 1; end
    wait_frames(3);
    a1 = amp_pp(1'b0);
    if (a1 >= 63 && a1 <= 77) $display("PASS: +1 scale amplitude = %0d px", a1);
    else begin $display("FAIL: +1 amplitude = %0d px (expect ~70)", a1); errors = errors + 1; end

    // 再顺时针一格 → soff +2
    detent4(1);
    #10000;
    if ($signed(vd_cap[8:6]) == 2) $display("PASS: view_data soff A -> +2");
    else begin $display("FAIL: soff A = %0d (expect 2)", $signed(vd_cap[8:6])); errors = errors + 1; end
    wait_frames(3);
    a2 = amp_pp(1'b0);
    if (a2 >= 43 && a2 <= 57) $display("PASS: +2 scale amplitude = %0d px", a2);
    else begin $display("FAIL: +2 amplitude = %0d px (expect ~50)", a2); errors = errors + 1; end

    // 逆时针两格 → 回自动
    detent4(0); detent4(0);
    #10000;
    wait_frames(3);
    a3 = amp_pp(1'b0);
    if ($signed(vd_cap[8:6]) == 0 && a3 >= 94 && a3 <= 106)
        $display("PASS: back to auto scale, amplitude = %0d px", a3);
    else begin $display("FAIL: soff=%0d amp=%0d", $signed(vd_cap[8:6]), a3); errors = errors + 1; end

    // 按压复位：从 +2 偏移一键回自动
    detent4(1); detent4(1);
    #10000;
    enc4_sw = 0; #20_000_000; enc4_sw = 1;     // 按压 20ms（含消抖）
    #10000;
    if ($signed(vd_cap[8:6]) == 0 && $signed(vd_cap[5:3]) == 0 && $signed(vd_cap[2:0]) == 0)
        $display("PASS: ENC4_SW resets offsets to auto");
    else begin $display("FAIL: after SW soff = %0d/%0d/%0d", $signed(vd_cap[8:6]), $signed(vd_cap[5:3]), $signed(vd_cap[2:0])); errors = errors + 1; end
    wait_frames(3);
    a3 = amp_pp(1'b0);
    if (a3 >= 94 && a3 <= 106) $display("PASS: post-reset amplitude = %0d px", a3);
    else begin $display("FAIL: post-reset amplitude = %0d", a3); errors = errors + 1; end

    if (errors == 0) $display("=== enc4_chain_tb ALL PASS ===");
    else             $display("=== enc4_chain_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #600_000_000;
    $display("TIMEOUT: enc4_chain_tb (errors=%0d)", errors);
    $finish;
end

endmodule
