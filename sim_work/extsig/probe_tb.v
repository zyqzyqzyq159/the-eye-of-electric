// 探针 TB：AD7928 模型 + 显示域 wf 握手全程监视
`timescale 1ns / 1ps
module probe_tb;

reg clk100 = 1'b0;
reg clk74  = 1'b0;
always #5.000 clk100 = ~clk100;              // 100MHz
always #6.734 clk74  = ~clk74;               // 74.25MHz

wire        ADC_SCLK, ADC_CS_N, ADC_MOSI;
wire        vid_hs, vid_vs, vid_de;
wire [7:0]  rgb_r, rgb_g, rgb_b;
wire [7:0]  LED;
wire        WS2812_DIN, BUZZER;

reg         BTN_FUNC = 1'b1, BTN_VIEW = 1'b1, BTN_FREEZE = 1'b1;
reg         BTN_SEL1 = 1'b1, BTN_SEL2 = 1'b1, BTN_SEL3 = 1'b1;
reg         BTN_SW2 = 1'b0, BTN_SW3 = 1'b0;   // user_input 视 SW2/SW3 为高有效：空闲须为 0
reg         ENC1_CHA = 1'b1, ENC1_CHB = 1'b1, ENC1_SW = 1'b1;
reg         ENC2_CHA = 1'b1, ENC2_CHB = 1'b1;
reg         ENC3_CHA = 1'b1, ENC3_CHB = 1'b1, ENC3_SW = 1'b1;
reg         ENC4_CHA = 1'b1, ENC4_CHB = 1'b1, ENC4_SW = 1'b1;

reg         adc_dout;                        // 模型 DOUT

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

// ===========================================================================
// AD7928 行为模型（与 adc_if_tb 相同的逐位模型；输入为连续时间正弦）
// ===========================================================================
reg  dbg_en = 0;
real sig_amp = 819.0;
real sig_dc  = 2100.0;
real sig_freq = 50.000;

function integer ch_input;
    input [2:0] ch;
    real t_us, ang, cyc;
    integer v;
    begin
        if (ch == 3'd0) begin
            t_us = $realtime / 1000.0;
            cyc = sig_freq * t_us * 1.0e-6;              // 周期数（double）
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
integer     dout_bit, fall_cnt;
reg  [11:0] din_shift, frame_val;

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
    frame_val = ch_input(conv_chan);
    dout_frame = {1'b0, conv_chan, frame_val};
    #30 adc_dout = dout_frame[15];
end

always @(posedge ADC_CS_N) begin
    adc_dout  = 1'bz;
    conv_chan = ctrl_reg[8:6];
end

// ===========================================================================
// 监视：100MHz 侧源脉冲/翻转计数 + 显示域握手信号逐变记录
// ===========================================================================
integer f3;
integer sp_cnt = 0, tog_cnt = 0;
reg wf_prev = 1'b0;
reg [3:0] dbg_prev = 4'hF;
wire [3:0] dbg_bus;
assign dbg_bus = {dut.display_ctrl_inst.wf_sample,
                  dut.display_ctrl_inst.wf_tg_p,
                  dut.display_ctrl_inst.wf_tg_s,
                  dut.display_ctrl_inst.wf_tg_m};

always @(posedge clk100) begin
    if (dut.eff_sample_pulse) begin
        sp_cnt = sp_cnt + 1;
        if (dbg_en) $fwrite(f3, "%0t SP func=%0d frzsrc=%0d
", $realtime,
                            dut.func_mode, dut.freeze);
    end
    if (dut.wf_toggle !== wf_prev) begin
        tog_cnt = tog_cnt + 1;
        wf_prev = dut.wf_toggle;
    end
    if (dbg_en && dbg_bus !== dbg_prev) begin
        $fwrite(f3, "%0t bus=%b frz=%b tg=%b\n", $realtime, dbg_bus,
                dut.display_ctrl_inst.frz_s, dut.wf_toggle);
        dbg_prev <= dbg_bus;
    end
end

integer disp_cnt = 0;
always @(posedge clk74) begin
    if (dbg_en && dut.display_ctrl_inst.wf_sample) disp_cnt = disp_cnt + 1;
end

initial begin
    ctrl_reg = 12'h000; conv_chan = 3'd0; adc_dout = 1'bz;
    dout_frame = 16'd0; dout_bit = 14; fall_cnt = 0; din_shift = 12'd0;
    f3 = $fopen("probe_wf.txt", "w");
    #200_000;
    #5_000_000;
    BTN_FUNC = 1'b0; #10_000_000; BTN_FUNC = 1'b1; #1_000_000;
    #20_000_000;
    BTN_FUNC = 1'b0; #10_000_000; BTN_FUNC = 1'b1; #1_000_000;
    #20_000_000;                          // t ≈ 67ms
    dbg_en = 1;
    #8_000_000;                           // 记录 8ms 窗口
    dbg_en = 0;
    $display("src pulses=%0d toggles=%0d disp wf_sample seen=%0d", sp_cnt, tog_cnt, disp_cnt);
    $display("pll_locked=%0d freq=%0d", dut.pll_locked_sync, dut.freq_out);
    $fclose(f3);
    $finish;
end

endmodule
