`timescale 1ns / 1ps
// 自检链路验证：dds_signal -> 采样 -> fft_top -> measurement + pll_sync 频率
// 完全复刻 top.v 中 DIP[0]=1 自检模式的接线（6 通道：V 0-2 / I 3-5），
// 检查 RMS/THD/FREQ/UNB/SEQ/V-I 相位角实际输出
// 五阶段：1 纯净 / 2 失真(DIP1) / 3 不平衡(DIP2) / 4 失真+不平衡 / 5 逆相序(DIP3)
//
// 标定基准（measurement.v RMS_MUL_V=2828 / RMS_MUL_I=1129，9-22 幅值自洽版）：
//   满幅电压 2047 code 峰 ↔ 一次侧 50V 峰 = 35.36V RMS → 读数 ~354（0.1V，=50/√2）
//   满幅电流 2047 code 峰 ↔ 20.00A 峰（G=0.947 设计）= 14.14A RMS → 读数 ~141（0.1A，=20/√2）
//   自检 DDS 固定满幅：相位/THD/不平衡/谐波百分比与幅值无关，仅 RMS 随注入变化
module selfcheck_chain_tb;

reg clk, rst_n;
reg [3:0] DIP;
reg distort, unbal, rev;

wire [11:0] dds_va, dds_vb, dds_vc;
wire [11:0] dds_ia, dds_ib, dds_ic;
wire        dds_valid;

dds_signal #(
    .CLK_FREQ(100_000_000), .DAC_BITS(12), .LUT_SIZE(256)
) dds_inst (
    .clk(clk), .rst_n(rst_n),
    .dds_va(dds_va), .dds_vb(dds_vb), .dds_vc(dds_vc),
    .dds_ia(dds_ia), .dds_ib(dds_ib), .dds_ic(dds_ic), .dds_valid(dds_valid),
    .freq_config(32'd50), .amp_config(16'd2048),
    .enable(DIP[0]), .select(DIP[0]),
    .distort_en(distort), .unbal_en(unbal), .rev_seq(rev)
);

// 与 top.v 相同的 PLL 接法：自检模式锁 DDS-A，验证频率真读数
wire [31:0] pll_freq;
wire        pll_locked;
wire [15:0] pll_phase;

pll_sync #(
    .CLK_FREQ(100_000_000), .SAMPLES_PER_CYCLE(256), .ADC_BITS(12)
) pll_inst (
    .clk(clk), .rst_n(rst_n),
    .adc_data(dds_va), .adc_channel(3'd0), .adc_valid(dds_valid),
    .sample_pulse(), .freq_out(pll_freq), .phase_out(pll_phase),
    .locked(pll_locked), .enable(1'b1)
);

// 与 top.v 相同的采样脉冲：100MHz/7813 = 12.8kHz
reg [15:0] dds_samp_cnt;
wire dds_samp_pulse;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) dds_samp_cnt <= 16'd0;
    else if (dds_samp_cnt >= 16'd7812) dds_samp_cnt <= 16'd0;
    else dds_samp_cnt <= dds_samp_cnt + 1'b1;
end
assign dds_samp_pulse = (dds_samp_cnt == 16'd7812);

wire eff_sample_pulse = dds_samp_pulse;   // DIP[0]=1

// FFT 通道轮换（与 top.v 一致）：0=Va→1=Vb→2=Vc→3=Ia→4=Ib→5=Ic
reg [2:0] fft_chan_sel;
wire [31:0] fft_mag;
wire [15:0] fft_phase;
wire [7:0]  fft_bin;
wire        fft_done;
wire [2:0]  fft_channel;
wire        fft_mag_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) fft_chan_sel <= 3'd0;
    else if (fft_done) fft_chan_sel <= (fft_chan_sel == 3'd5) ? 3'd0 : fft_chan_sel + 3'd1;
end

wire [11:0] dds_fft_v = (fft_chan_sel == 3'd0) ? dds_va :
                        (fft_chan_sel == 3'd1) ? dds_vb : dds_vc;
wire [11:0] dds_fft_i = (fft_chan_sel == 3'd3) ? dds_ia :
                        (fft_chan_sel == 3'd4) ? dds_ib : dds_ic;
wire [11:0] fft_src = (fft_chan_sel < 3'd3) ? dds_fft_v : dds_fft_i;

reg [11:0] fft_data_reg;
reg        fft_valid_reg;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fft_data_reg <= 12'd0;
        fft_valid_reg <= 1'b0;
    end else begin
        fft_valid_reg <= 1'b0;
        if (eff_sample_pulse) begin
            fft_data_reg <= fft_src;
            fft_valid_reg <= 1'b1;
        end
    end
end

fft_top #(
    .FFT_SIZE(256), .DATA_BITS(12)
) fft_inst (
    .clk(clk), .rst_n(rst_n),
    .data_in(fft_data_reg), .channel_in(fft_chan_sel), .valid_in(fft_valid_reg),
    .sample_sync(eff_sample_pulse),
    .mag_out(fft_mag), .phase_out(fft_phase), .bin_index(fft_bin),
    .fft_done(fft_done), .fft_channel(fft_channel), .mag_valid(fft_mag_valid),
    .enable(1'b1)
);

wire [31:0] fundamental_freq;
wire [31:0] rms_va, rms_vb, rms_vc;
wire [31:0] rms_ia, rms_ib, rms_ic;
wire [31:0] thd_va, thd_vb, thd_vc;
wire [31:0] thd_ia, thd_ib, thd_ic;
wire [239:0] harm_pct_va, harm_pct_vb, harm_pct_vc;
wire [239:0] harm_pct_ia, harm_pct_ib, harm_pct_ic;
wire [15:0] phase_ab, phase_bc, phase_ca;
wire [15:0] phase_ia, phase_ib, phase_ic;
wire        phase_sequence;
wire [31:0] unbalance_degree;
wire        measure_done, done_toggle;

wire [177:0] pwr_bus;
reg [14:0] pf_a_v;
reg [15:0] pa_v, qa_v, ptot_v, qtot_v, cap_v;
reg cpres_v, pfok_v, lag_v;

measurement #(
    .FFT_SIZE(256), .HARMONICS_NUM(15)
) meas_inst (
    .clk(clk), .rst_n(rst_n),
    .mag_in(fft_mag), .phase_in(fft_phase), .bin_in(fft_bin),
    .mag_valid(fft_mag_valid), .fft_done(fft_done), .fft_channel(fft_channel),
    .fundamental_freq(fundamental_freq),
    .rms_va(rms_va), .rms_vb(rms_vb), .rms_vc(rms_vc),
    .rms_ia(rms_ia), .rms_ib(rms_ib), .rms_ic(rms_ic),
    .thd_va(thd_va), .thd_vb(thd_vb), .thd_vc(thd_vc),
    .thd_ia(thd_ia), .thd_ib(thd_ib), .thd_ic(thd_ic),
    .harm_pct_va(harm_pct_va), .harm_pct_vb(harm_pct_vb), .harm_pct_vc(harm_pct_vc),
    .harm_pct_ia(harm_pct_ia), .harm_pct_ib(harm_pct_ib), .harm_pct_ic(harm_pct_ic),
    .phase_ab(phase_ab), .phase_bc(phase_bc), .phase_ca(phase_ca),
    .phase_ia(phase_ia), .phase_ib(phase_ib), .phase_ic(phase_ic),
    .phase_sequence(phase_sequence), .unbalance_degree(unbalance_degree),
    .pwr_bus(pwr_bus),
    .measure_done(measure_done), .done_toggle(done_toggle),
    .enable(1'b1)
);

integer done_cnt;
reg done_tg_prev;
always @(posedge clk) begin
    done_tg_prev <= done_toggle;
    if (done_toggle != done_tg_prev) begin
        done_cnt = done_cnt + 1;  // 数翻转沿，不数电平
        if (done_cnt > 1)
            $display("[round %0d @ %0t us] rmsV=%0d/%0d/%0d rmsI=%0d/%0d/%0d thdV=%0d thdI=%0d unb=%0d ph_ia=%0d seq=%b pll=%0d",
                     done_cnt, $time/1000,
                     rms_va, rms_vb, rms_vc, rms_ia, rms_ib, rms_ic,
                     thd_va, thd_ia, unbalance_degree, phase_ia, phase_sequence, pll_freq);
    end
end

// 相位差判定：-120° = -21845（65536=360°）。
// 容差 ±1° = ±182：残差来自 DDS 256 点 LUT 阶梯量化（1.4°/步进，各相帧起点
// 落在表内不同位置产生固定偏置）；理想正弦时相位差 = -120.03°。
// 逆相序时相位差 = +120° = +21845。V→I 相位角：30° 滞后 = -5461，容差 ±2.5°=±455。
wire signed [15:0] pab_s = $signed(phase_ab);
wire signed [15:0] pbc_s = $signed(phase_bc);
wire signed [15:0] pca_s = $signed(phase_ca);
wire signed [15:0] pia_s = $signed(phase_ia);
wire signed [15:0] pib_s = $signed(phase_ib);
wire signed [15:0] pic_s = $signed(phase_ic);

initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
end

integer i;
initial begin
    rst_n  = 1'b0;
    DIP    = 4'b0001;
    distort = 1'b0;
    unbal   = 1'b0;
    rev     = 1'b0;
    done_cnt = 0;
    done_tg_prev = 1'b0;
    #1000;
    rst_n = 1'b1;

    // 一轮测量 = 6 通道 × 512 采样脉冲 = 240ms（帧对齐后）= 6 个 done 翻转。
    // 每阶段切换后等 2 整轮（12 翻转）：measurement 链式归属会顺带刷新
    // cur_phase..5 的旧通道（mag_buf 最多滞后一整轮），且切换瞬间的在途帧
    // 是混合帧必须跳过——3 通道时代等 4 翻转的旧参数对 6 通道不够。
    wait (done_cnt >= 12);
    #1000;

    // ======== 阶段 1：纯净正弦（DIP1/2/3=0）========
    $display("=== Phase 1: pure sine (after %0d rounds) ===", done_cnt);
    $display("rmsV A/B/C = %0d / %0d / %0d  (0.1V, expect ~354 = 35.4V 满幅)", rms_va, rms_vb, rms_vc);
    $display("rmsI A/B/C = %0d / %0d / %0d  (0.1A, expect ~141 = 14.1A 满幅)", rms_ia, rms_ib, rms_ic);
    $display("thd  A V/I = %0d / %0d  (permille, expect ~0)", thd_va, thd_ia);
    $display("freq(pll) = %0d (0.1Hz, expect ~499; DDS 相位增量取整实际 49.9887Hz)", pll_freq);
    $display("phase ab/bc/ca = %0d / %0d / %0d (signed, -21845 = -120deg)", pab_s, pbc_s, pca_s);
    $display("phase V-I a/b/c = %0d / %0d / %0d (signed, -5461 = -30deg 感性)", pia_s, pib_s, pic_s);
    if (pab_s <= -16'sd21663 && pab_s >= -16'sd22027 &&
        pbc_s <= -16'sd21663 && pbc_s >= -16'sd22027 &&
        pca_s <= -16'sd21663 && pca_s >= -16'sd22027)
        $display("PASS: all phase differences = -120deg +/- 1deg (frame alignment OK)");
    else
        $display("FAIL: phase differences deviate from -120deg");
    if (pia_s <= -16'sd5006 && pia_s >= -16'sd5916 &&
        pib_s <= -16'sd5006 && pib_s >= -16'sd5916 &&
        pic_s <= -16'sd5006 && pic_s >= -16'sd5916)
        $display("PASS: V-I phase angles = -30deg +/- 2.5deg (inductive load model OK)");
    else
        $display("FAIL: V-I phase angles deviate from -30deg");
    if (rms_va >= 32'd349 && rms_va <= 32'd359 &&
        rms_ia >= 32'd136  && rms_ia <= 32'd146)
        $display("PASS: full-scale RMS calibration V=35.4V I=14.1A");
    else
        $display("FAIL: RMS calibration (V expect 349-359, I expect 136-146)");
    $display("seq = %b (expect 1 = ABC)  unbalance = %0d (expect 0)", phase_sequence, unbalance_degree);
    $display("harm A: h1=%0d h2=%0d h3=%0d (permille)", harm_pct_va[15:0], harm_pct_va[31:16], harm_pct_va[47:32]);
    $display("harmI A: h1=%0d h2=%0d h3=%0d (permille)", harm_pct_ia[15:0], harm_pct_ia[31:16], harm_pct_ia[47:32]);

    // ======== 阶段 2：失真注入（DIP1=1）========
    // 注入 20% 三次 + 10% 五次谐波：预期 h3=200‰ h5=100‰，其余谐波≈0；
    // THD 读数按本设计线性求和定义 = h3+h5+... ≈ 300‰
    //（标准均方根定义理论值 sqrt(0.2^2+0.1^2) = 224‰）
    distort = 1'b1;
    wait (done_cnt >= 24);     // 切换后 2 整轮，在途混合帧已跳过
    #1000;

    $display("=== Phase 2: distorted sine, 20%% 3rd + 10%% 5th (after %0d rounds) ===", done_cnt);
    $display("thd  A V/I = %0d / %0d  (permille, sum-def expect ~300)", thd_va, thd_ia);
    $display("harm A: h1=%0d h2=%0d h3=%0d h4=%0d h5=%0d h6=%0d (permille)",
             harm_pct_va[15:0],  harm_pct_va[31:16], harm_pct_va[47:32],
             harm_pct_va[63:48], harm_pct_va[79:64], harm_pct_va[95:80]);
    $display("rmsV A = %0d  rmsI A = %0d (幅值和定义 expect ~6620 / ~185)", rms_va, rms_ia);
    $display("freq = %0d Hz (expect 50)", fundamental_freq);

    // 判定：h3≈200 h5≈100，THD≈300（容差覆盖 LUT 量化与 FFT 舍入）
    if (thd_va >= 32'd284 && thd_va <= 32'd316 &&
        harm_pct_va[47:32] >= 16'd192 && harm_pct_va[47:32] <= 16'd208 &&
        harm_pct_va[79:64] >= 16'd92  && harm_pct_va[79:64] <= 16'd108 &&
        harm_pct_va[31:16] <= 16'd8   && harm_pct_va[63:48] <= 16'd8)
        $display("PASS: harmonic analysis detects 20%% 3rd + 10%% 5th correctly");
    else
        $display("FAIL: harmonic percentages deviate from injected distortion");
    if (thd_ia >= 32'd284 && thd_ia <= 32'd316 &&
        harm_pct_ia[47:32] >= 16'd192 && harm_pct_ia[47:32] <= 16'd208 &&
        harm_pct_ia[79:64] >= 16'd92  && harm_pct_ia[79:64] <= 16'd108)
        $display("PASS: current harmonics track voltage injection");
    else
        $display("FAIL: current harmonics deviate");

    // ======== 阶段 3：不平衡注入（DIP2=1，纯净）========
    // A/B/C = 100%/80%/60%：rmsV 比例 353/283/212；不平衡度 = max偏差/平均 = 250‰
    distort = 1'b0;
    unbal   = 1'b1;
    wait (done_cnt >= 36);
    #1000;

    $display("=== Phase 3: unbalance A/B/C=100/80/60%% (after %0d rounds) ===", done_cnt);
    $display("rmsV A/B/C = %0d / %0d / %0d  (0.1V, expect ~354/~284/~213)", rms_va, rms_vb, rms_vc);
    $display("rmsI A/B/C = %0d / %0d / %0d  (0.1A, expect ~141/~113/~85)", rms_ia, rms_ib, rms_ic);
    $display("unbalance = %0d (permille, expect ~250 = 0.2/0.8)", unbalance_degree);
    $display("thd  A V/I = %0d / %0d  (expect 0)", thd_va, thd_ia);
    if (rms_va >= 32'd348 && rms_va <= 32'd358 &&
        rms_vb >= 32'd278 && rms_vb <= 32'd288 &&
        rms_vc >= 32'd207 && rms_vc <= 32'd217)
        $display("PASS: per-phase RMS ratios 100/80/60 detected");
    else
        $display("FAIL: RMS ratios deviate from 100/80/60");
    if (rms_ia >= 32'd136 && rms_ia <= 32'd146 &&
        rms_ib >= 32'd107 && rms_ib <= 32'd119 &&
        rms_ic >= 32'd79  && rms_ic <= 32'd91)
        $display("PASS: current RMS ratios 100/80/60 detected");
    else
        $display("FAIL: current RMS ratios deviate");
    if (unbalance_degree >= 32'd200 && unbalance_degree <= 32'd300)
        $display("PASS: unbalance measurement ~250 permille");
    else
        $display("FAIL: unbalance deviates from 250 permille");
    if (thd_va <= 32'd10 && thd_ia <= 32'd10)
        $display("PASS: pure sines, no spurious THD");
    else
        $display("FAIL: spurious THD with unbalanced amplitudes");

    // ======== 阶段 4：失真 + 不平衡（DIP1+DIP2）========
    // 谐波随幅值同比例缩放 → 各相 h3/fund = 20%、h5/fund = 10%、THD = 300‰ 一致
    distort = 1'b1;
    wait (done_cnt >= 48);
    #1000;

    $display("=== Phase 4: distortion + unbalance (after %0d rounds) ===", done_cnt);
    $display("thd  A/B/C V = %0d / %0d / %0d  (all expect ~300)", thd_va, thd_vb, thd_vc);
    $display("thd  A/B/C I = %0d / %0d / %0d  (all expect ~300)", thd_ia, thd_ib, thd_ic);
    $display("harm B: h3=%0d h5=%0d   harm C: h3=%0d h5=%0d (permille, expect ~200/~100)",
             harm_pct_vb[47:32], harm_pct_vb[79:64], harm_pct_vc[47:32], harm_pct_vc[79:64]);
    $display("unbalance = %0d (expect ~250)", unbalance_degree);
    if (thd_va >= 32'd284 && thd_va <= 32'd316 &&
        thd_vb >= 32'd284 && thd_vb <= 32'd316 &&
        thd_vc >= 32'd284 && thd_vc <= 32'd316 &&
        thd_ia >= 32'd284 && thd_ia <= 32'd316 &&
        thd_ib >= 32'd284 && thd_ib <= 32'd316 &&
        thd_ic >= 32'd284 && thd_ic <= 32'd316)
        $display("PASS: THD ratios independent of amplitude (scaled LUTs OK)");
    else
        $display("FAIL: THD ratios deviate under unbalance");
    if (unbalance_degree >= 32'd200 && unbalance_degree <= 32'd300)
        $display("PASS: unbalance still ~250 with distortion");
    else
        $display("FAIL: unbalance wrong in combined mode");

    // ======== 阶段 5：逆相序（DIP3=1，B/C 互换）========
    distort = 1'b0;
    unbal   = 1'b0;
    rev     = 1'b1;
    wait (done_cnt >= 60);
    #1000;

    $display("=== Phase 5: reverse sequence B<->C (after %0d rounds) ===", done_cnt);
    $display("seq = %b (expect 0 = ACB)", phase_sequence);
    $display("phase ab/bc/ca = %0d / %0d / %0d (signed, +21845 = +120deg)", pab_s, pbc_s, pca_s);
    $display("rmsV A/B/C = %0d / %0d / %0d  (expect ~5050 x3)", rms_va, rms_vb, rms_vc);
    if (phase_sequence == 1'b0)
        $display("PASS: phase sequence detection flipped to ACB");
    else
        $display("FAIL: phase sequence still reads ABC");
    if (pab_s >= 16'sd21663 && pab_s <= 16'sd22027 &&
        pbc_s >= 16'sd21663 && pbc_s <= 16'sd22027 &&
        pca_s >= 16'sd21663 && pca_s <= 16'sd22027)
        $display("PASS: phase differences = +120deg (reverse sequence)");
    else
        $display("FAIL: phase differences not +120deg");
    if (pia_s <= -16'sd5006 && pia_s >= -16'sd5916)
        $display("PASS: V-I angle still -30deg for phase A under reverse sequence");
    else
        $display("FAIL: V-I angle wrong under reverse sequence");

    // ======== PLL 频率总结（自检模式真频率）========
    $display("=== PLL frequency (self-check, real measurement) ===");
    $display("pll_freq = %0d (0.1Hz)  locked = %b  (DDS true freq 49.9887Hz -> 499)", pll_freq, pll_locked);
    if (pll_locked && pll_freq >= 32'd497 && pll_freq <= 32'd502)
        $display("PASS: PLL tracks DDS-A, real frequency display verified");
    else
        $display("FAIL: PLL frequency out of range");

    begin
        // ---- 功率/PF/补偿（DDS 电流滞后 30°：PF=0.866，滞后=感性→建议并电容）----
        pf_a_v   = pwr_bus[177:163];
        pa_v     = pwr_bus[162:147];
        qa_v     = pwr_bus[114:99];
        ptot_v   = pwr_bus[66:51];
        qtot_v   = pwr_bus[50:35];
        cap_v    = pwr_bus[34:19];
        cpres_v  = pwr_bus[2];
        pfok_v   = pwr_bus[1];
        lag_v    = pwr_bus[0];
        $display("PF=%0d/16384 (%0.3f)  Pa=%0dW  Qa=%0dvar  Ptot=%0dW  Qtot=%0dvar",
                 pf_a_v, pf_a_v/16384.0, pa_v, qa_v, ptot_v, qtot_v);
        $display("[dbg] state=%0d pw_pfa=%0d pw_p0=%0d pw_q0=%0d cos=%0d vi=%0d phiA=%04h",
                 meas_inst.state, meas_inst.pw_pfa_r, meas_inst.pw_p[0], meas_inst.pw_q[0], meas_inst.pw_cos_r, meas_inst.pw_vi_r, meas_inst.phase_ia_reg);
        $display("cur_pres=%b pf_ok=%b lag=%b  cap=%0d uF  ind=%0d (0.1mH)",
                 cpres_v, pfok_v, lag_v, cap_v, pwr_bus[18:3]);
        if (pf_a_v >= 15'sd14000 && pf_a_v <= 15'sd14400)
            $display("PASS: PF = 0.866 (cos 30deg)");
        else
            $display("FAIL: PF %0d (expect ~14196)", pf_a_v);
        if (pa_v >= 16'd420 && pa_v <= 16'd450 && qa_v >= 16'd235 && qa_v <= 16'd265)
            $display("PASS: Pa = V*I*cos30 = ~434W, Qa = V*I*sin30 = ~250var");
        else
            $display("FAIL: P=%0d Q=%0d (expect ~434/~250)", pa_v, qa_v);
        if (cpres_v === 1'b1 && lag_v === 1'b1 && pfok_v === 1'b0 &&
            cap_v >= 16'd50 && cap_v <= 16'd70)
            $display("PASS: inductive detected, suggest CAP ~60uF");
        else
            $display("FAIL: flags cpres=%b lag=%b pfok=%b cap=%0d", cpres_v, lag_v, pfok_v, cap_v);
    end

    $finish;
end

initial begin
    repeat (10) #1_000_000_000;   // 10s 总超时，拆分避免 32bit 有符号溢出
    $display("TIMEOUT");
    $finish;
end

endmodule
