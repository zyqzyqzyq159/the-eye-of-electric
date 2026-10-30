// 定向探针 TB：s9 坏值风暴下的 PLL 过零事件逐个记录
// ——定位 V26 vs V25 在风暴中段失锁差异的机理（zc 事件相位/输入值/err）
`timescale 1ns / 1ps

module zcprobe_tb;

reg clk100 = 1'b0;
reg clk74  = 1'b0;
always #5.000 clk100 = ~clk100;
always #6.734 clk74  = ~clk74;

wire        ADC_SCLK, ADC_CS_N, ADC_MOSI;
wire        vid_hs, vid_vs, vid_de;
wire [7:0]  rgb_r, rgb_g, rgb_b;
wire [7:0]  LED;
wire        WS2812_DIN, BUZZER;

reg         BTN_FUNC = 1'b1, BTN_VIEW = 1'b1, BTN_FREEZE = 1'b1;
reg         BTN_SEL1 = 1'b1, BTN_SEL2 = 1'b1, BTN_SEL3 = 1'b1;
reg         BTN_SW2 = 1'b0, BTN_SW3 = 1'b0;
reg         ENC1_CHA = 1'b1, ENC1_CHB = 1'b1, ENC1_SW = 1'b1;
reg         ENC2_CHA = 1'b1, ENC2_CHB = 1'b1;
reg         ENC3_CHA = 1'b1, ENC3_CHB = 1'b1, ENC3_SW = 1'b1;
reg         ENC4_CHA = 1'b1, ENC4_CHB = 1'b1, ENC4_SW = 1'b1;

reg        adc_dout;

top_sim #(.CLK_FREQ(100_000_000)) dut (
    .sim_clk_100m(clk100), .sim_clk_74m(clk74),
    .ADC_SCLK(ADC_SCLK), .ADC_CS_N(ADC_CS_N), .ADC_MISO(adc_dout), .ADC_MOSI(ADC_MOSI),
    .vid_hs(vid_hs), .vid_vs(vid_vs), .vid_de(vid_de),
    .rgb_r(rgb_r), .rgb_g(rgb_g), .rgb_b(rgb_b),
    .LED(LED),
    .BTN_FUNC(BTN_FUNC), .BTN_VIEW(BTN_VIEW), .BTN_FREEZE(BTN_FREEZE),
    .BTN_SEL1(BTN_SEL1), .BTN_SEL2(BTN_SEL2), .BTN_SEL3(BTN_SEL3),
    .BTN_SW2(BTN_SW2), .BTN_SW3(BTN_SW3),
    .ENC1_CHA(ENC1_CHA), .ENC1_CHB(ENC1_CHB), .ENC1_SW(ENC1_SW),
    .ENC2_CHA(ENC2_CHA), .ENC2_CHB(ENC2_CHB),
    .ENC3_CHA(ENC3_CHA), .ENC3_CHB(ENC3_CHB), .ENC3_SW(ENC3_SW),
    .ENC4_CHA(ENC4_CHA), .ENC4_CHB(ENC4_CHB), .ENC4_SW(ENC4_SW),
    .WS2812_DIN(WS2812_DIN), .BUZZER(BUZZER)
);

// ==== AD7928 模型（与 extsig_tb 相同：定向 A 相坏值注入）====
real  sig_amp;
real  sig_dc;
real  sig_freq;
reg   stray_en = 0;
integer frame_ctr = 0;
integer a_ctr = 0;

function integer ch_input;
    input [2:0] ch;
    real t_us, ang, cyc;
    integer v;
    begin
        if (ch == 3'd0) begin
            t_us = $realtime / 1000.0;
            cyc = sig_freq * t_us * 1.0e-6;
            ang  = 6.283185307179586 * (cyc - $rtoi(cyc));
            v = $rtoi(sig_dc + sig_amp * $sin(ang));
            if (v < 0)    v = 0;
            if (v > 4095) v = 4095;
            ch_input = v;
        end else begin
            ch_input = 2048;
        end
    end
endfunction

reg  [11:0] ctrl_reg;
reg  [2:0]  conv_chan;
reg  [15:0] dout_frame;
integer     dout_bit;
integer     fall_cnt;
reg  [11:0] din_shift;
reg  [11:0] frame_val;

always @(negedge ADC_SCLK) if (!ADC_CS_N) begin
    fall_cnt = fall_cnt + 1;
    if (fall_cnt <= 12) begin
        din_shift = {din_shift[10:0], ADC_MOSI};
        if (fall_cnt == 12 && din_shift[11] === 1'b1) ctrl_reg = din_shift;
    end
    if (fall_cnt <= 15) begin
        #40 adc_dout = dout_frame[dout_bit];
        dout_bit = dout_bit - 1;
    end else begin
        #20 adc_dout = 1'bz;
    end
end

always @(negedge ADC_CS_N) begin
    fall_cnt  = 0;
    dout_bit  = 14;
    adc_dout  = 1'bz;
    frame_ctr = frame_ctr + 1;
    if (conv_chan == 3'd0) a_ctr = a_ctr + 1;
    frame_val = ch_input(conv_chan);
    if (stray_en && (a_ctr % 120) == 7  && conv_chan == 3'd0) frame_val = 12'd4095;
    if (stray_en && (a_ctr % 120) == 67 && conv_chan == 3'd0) frame_val = 12'd0;
    dout_frame = {1'b0, conv_chan, frame_val};
    #30 adc_dout = dout_frame[15];
end

always @(posedge ADC_CS_N) begin
    adc_dout  = 1'bz;
    conv_chan = ctrl_reg[8:6];
end

// ==== zc 事件探针：每次 zc_rise 记录时刻 / PLL输入 / 理想信号值 / NCO 相位 ====
integer zc_f;
integer ev_cnt = 0;
reg probe_on = 0;
real t_us, cyc, frac, ideal_v, phase_deg;

// PLL A 帧输入序列探针
integer pll_f;
initial pll_f = $fopen("pllseq_v26.txt", "w");
always @(posedge clk100) begin
    if (probe_on && dut.pll_val_r && dut.pll_chan_r == 3'd0)
        $fwrite(pll_f, "%0d
", dut.pll_data_r);
end


always @(posedge clk100) begin
    if (probe_on && dut.pll_sync_inst.zc_rise) begin
        t_us = $realtime / 1000.0;
        cyc = sig_freq * t_us * 1.0e-6;
        frac = cyc - $rtoi(cyc);
        ideal_v = sig_dc + sig_amp * $sin(6.283185307179586 * frac);
        // 理想过零点：frac=0（上升过零）。zc 事件的信号相位偏差：
        phase_deg = frac * 360.0;
        $fwrite(zc_f, "%0t,%0d,%0d,%0d,%0d,%.2f,%.2f,%0d,%0d\n",
                $time, ev_cnt,
                dut.pll_sync_inst.adc_curr, dut.pll_sync_inst.adc_prev,
                dut.ch_data[0], ideal_v, phase_deg,
                dut.pll_sync_inst.nco_phase[31:22], dut.pll_sync_inst.locked_reg);
        ev_cnt = ev_cnt + 1;
    end
end

task press_func;
    begin
        BTN_FUNC = 1'b0; #10_000_000; BTN_FUNC = 1'b1; #1_000_000;
    end
endtask

initial begin
    sig_amp = 819.0; sig_dc = 2263.0; sig_freq = 50.000;
    ctrl_reg = 12'h000; conv_chan = 3'd0; adc_dout = 1'bz;
    dout_frame = 16'd0; dout_bit = 14; fall_cnt = 0; din_shift = 12'd0;
    zc_f = $fopen("zcprobe_events.csv", "w");
    $fwrite(zc_f, "time_ns,ev,adc_curr,adc_prev,ch_data0,ideal_v,phase_deg,nco_phase_10bit,locked\n");
    #200_000;
    #5_000_000;
    press_func; #20_000_000;
    press_func; #5_000_000;
    // 等锁定 + 稳定
    #150_000_000;
    probe_on = 1; stray_en = 1;
    #300_000_000;                 // 300ms 风暴窗口逐事件记录
    probe_on = 0; stray_en = 0;
    // 稳态读数复查：风暴后再跑 300ms 干净信号，每 100ms 打印 rms/thd
    #300_000_000;
    $display("[steady] rms_va=%0d thd_va=%0d freq=%0d dc=%0d",
             dut.rms_va, dut.thd_va, dut.freq_out, dut.dc_bias_c);
    #100_000_000;
    $display("[steady] rms_va=%0d thd_va=%0d freq=%0d dc=%0d",
             dut.rms_va, dut.thd_va, dut.freq_out, dut.dc_bias_c);
    $display("zcprobe done, %0d events", ev_cnt);
    $fclose(zc_f);
    $finish;
end

initial begin
    #1_000_000_000;
    $display("TIMEOUT zcprobe_tb");
    $finish;
end

endmodule
