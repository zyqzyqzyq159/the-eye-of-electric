// top 的仿真替身：仅去掉 2 个 PLL IP 与 uihdmitx（时钟由 TB 直供、视频信号
// 直接引出），其余 adc_if/dds/dc_center/pll_sync/fft/measurement/display_ctrl/
// user_input/ws2812 与 top.v 逐行一致——用于复现"外接正弦波形畸变"
`timescale 1ns / 1ps
module top_sim #(
    parameter CLK_FREQ = 100_000_000
)(
    input  wire        sim_clk_100m,
    input  wire        sim_clk_74m,

    output wire        ADC_SCLK,
    output wire        ADC_CS_N,
    input  wire        ADC_MISO,
    output wire        ADC_MOSI,

    output wire        vid_hs, vid_vs, vid_de,
    output wire [7:0]  rgb_r, rgb_g, rgb_b,

    output wire [7:0]  LED,

    input  wire        BTN_FUNC,
    input  wire        BTN_VIEW,
    input  wire        BTN_FREEZE,
    input  wire        BTN_SEL1,
    input  wire        BTN_SEL2,
    input  wire        BTN_SEL3,
    input  wire        BTN_SW2,
    input  wire        BTN_SW3,

    input  wire        ENC1_CHA, ENC1_CHB, ENC1_SW,
    input  wire        ENC2_CHA, ENC2_CHB,
    input  wire        ENC3_CHA, ENC3_CHB, ENC3_SW,
    input  wire        ENC4_CHA, ENC4_CHB, ENC4_SW,

    output wire        WS2812_DIN,
    output wire        BUZZER
);

wire clk_100m = sim_clk_100m;
wire clk_74m  = sim_clk_74m;
wire pll_all_locked = 1'b1;                 // 仿真：PLL 恒锁定
wire rst_n;

reg [7:0]  rst_counter;
initial rst_counter = 8'd0;                  // 仿真：真机靠 extlock X→0 负沿清零
always @(posedge clk_100m or negedge pll_all_locked) begin
    if (!pll_all_locked) rst_counter <= 8'd0;
    else if (rst_counter < 8'd255) rst_counter <= rst_counter + 1'b1;
end
assign rst_n = (rst_counter >= 8'd200);

wire adc_run_en;
wire [11:0] adc_data;
wire [2:0]  adc_channel;
wire        adc_valid;

adc_if #(
    .CLK_FREQ(CLK_FREQ), .SPI_CLK_FREQ(12_500_000), .ADC_BITS(12), .CHANNELS(6)
) adc_if_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .adc_sclk(ADC_SCLK), .adc_cs_n(ADC_CS_N), .adc_miso(ADC_MISO), .adc_mosi(ADC_MOSI),
    .adc_data(adc_data), .adc_channel(adc_channel), .adc_valid(adc_valid),
    .enable(adc_run_en)
);

wire        sample_pulse;
wire [31:0] freq_out;
wire [15:0] phase_out;
wire        pll_locked_sync;

wire [11:0] dds_va, dds_vb, dds_vc;
wire [11:0] dds_ia, dds_ib, dds_ic;
wire        dds_valid;

wire [1:0]  func_mode;
wire [1:0]  disp_mode;
wire        freeze;
wire [2:0]  self_sel;
wire [2:0]  meas_chan;
wire [1:0]  meas_vi;
wire [1:0]  meas_pick;
wire [10:0] func_data;
wire        func_toggle;
wire [44:0] view_data;
wire        view_toggle;
wire        mode_change_pulse;
wire [9:0]  ui_event;

wire selftest_en = (func_mode == 2'd0);

dds_signal #(
    .CLK_FREQ(CLK_FREQ), .DAC_BITS(12), .LUT_SIZE(256)
) dds_signal_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .dds_va(dds_va), .dds_vb(dds_vb), .dds_vc(dds_vc),
    .dds_ia(dds_ia), .dds_ib(dds_ib), .dds_ic(dds_ic), .dds_valid(dds_valid),
    .freq_config(32'd50), .amp_config(16'd2048),
    .enable(selftest_en), .select(selftest_en),
    .distort_en(selftest_en & self_sel[0]),
    .unbal_en(selftest_en & self_sel[1]),
    .rev_seq(selftest_en & self_sel[2])
);

// V28 镜像 top.v：帧流三点中值滤波（全通道共享单套）
// --- 每通道保持寄存器 + 帧流三点中值滤波（V28，全通道，共享单套数据通路）---
// 0=Va 1=Vb 2=Vc（AMC1350 差分→偏置单端）3=Ia 4=Ib 5=Ic（CC6905 直出）
// 实机 ADC 帧流存在孤立坏帧（0~4095 任意码、随机相位，V19 实拍"每周期一根窄
// 毛刺"；含 SPI 位错与通道标记错位——标记错位时别通道的数据被写进本通道，
// 幅度可达满幅）。V26/V27 的 ±96 钳位只把坏帧"削减"为 ±96 残幅：残幅留在
// 波形上=断崖、进 FFT=THD/RMS 逐窗抖动、踢锁相过零=FREQ 乱跳；且 B/C/电流
// 通道完全无保护。三点中值彻底解决：
//   ① 孤立坏帧（任意幅度/任意通道）被完全滤除，不是削减——输出恒为真实样本；
//   ② 6 通道轮采下相邻帧必属不同通道 → 连续两个坏帧必落不同通道，每通道
//      视角永远是孤立坏帧，中值全杀；
//   ③ 对真实信号零损伤：同通道帧间隔 ~52µs，50Hz 满幅相邻帧差 ≤33 码、
//      750Hz 15 次谐波每周期仍有 25 帧——三点取中值=真实中间样本，仅整体
//      恒延 1 帧（52µs，对 PLL 只是常量相移，对 12.8kHz 采样无感）。
//   ④ 写法平凡（无符号比较+选择，V22.0 教训）；单套共享比较网络 ~+40 LUT。
// 上电前两帧历史初始化 2048，首帧中值=2048 无暂态。
reg [11:0] ch_data [0:5];
reg [11:0] med_in_r;                         // 拍1：帧数据/通道/有效锁存
reg [2:0]  med_ch_r;
reg        med_vd_r;
reg [11:0] med_h1 [0:5];                     // 每通道前一帧
reg [11:0] med_h2 [0:5];                     // 每通道前二帧
wire [11:0] med_a = med_h2[med_ch_r];
wire [11:0] med_b = med_h1[med_ch_r];
wire [11:0] med_c = med_in_r;
wire [11:0] med_out_w =
    (med_a > med_b) ? ((med_b > med_c) ? med_b :
                       (med_a > med_c) ? med_c : med_a) :
                      ((med_a > med_c) ? med_a :
                       (med_b > med_c) ? med_c : med_b);
integer ci;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        for (ci = 0; ci < 6; ci = ci + 1) begin
            ch_data[ci] <= 12'd2048;
            med_h1[ci]  <= 12'd2048;
            med_h2[ci]  <= 12'd2048;
        end
        med_in_r <= 12'd2048; med_ch_r <= 3'd0; med_vd_r <= 1'b0;
    end else begin
        med_in_r <= adc_data;
        med_ch_r <= adc_channel;
        med_vd_r <= adc_valid && (adc_channel < 3'd6);
        if (med_vd_r) begin
            ch_data[med_ch_r]   <= med_out_w;              // 中值输出
            med_h2[med_ch_r]    <= med_h1[med_ch_r];       // 历史滑动
            med_h1[med_ch_r]    <= med_in_r;
        end
    end
end

// --- 直流减除（V27：常数直减，弃自适应估计）---
// 采样板偏置为已知精确值：差放偏置芯片输出 1.318V → AD7928 码 = 1.318/2.5×4096
// = 2159.8 ≈ 2160。直接减常数，不做任何估计：
//   ① 断崖从根上消失——V17~V26 的自适应估计器每 20ms 窗口刷新估计值，显示
//      波形整窗跟着挪 ≤8 码，锁定后窗边界恰钉在过零相位 = 实机"零点微弱断崖"；
//      常数没有刷新，永远不动。
//   ② 数据更准——波形严格按链路数学居中；测量侧 RMS/THD 本就从 FFT bin1 起
//      天然无直流，与偏置无关；PLL 过零严格落在信号真零点（相位差/PF 无偏）。
//   ③ 省 ~150-250 LUT（6×20bit 累加器 + 限速器删除）。
// 若上板波形整体偏离中线（偏置芯片个体偏差），只需改 DC_BIAS_CODE 一个常数：
//   偏置码 = 偏置电压 ÷ 2.5 × 4096（1.318V→2160；历史档案记载 1.381V→2262）
localparam [11:0] DC_BIAS_CODE = 12'd2160;    // 1.318V 偏置的 ADC 码
wire [11:0] dc_bias_c = DC_BIAS_CODE;         // TB 探针引出
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

reg [11:0] pll_data_r;
reg [2:0]  pll_chan_r;
reg        pll_val_r;
reg pll_vd_d;
reg [2:0] pll_ch_d;
always @(posedge clk_100m) begin
    pll_vd_d   <= med_vd_r;
    pll_ch_d   <= med_ch_r;
    pll_data_r <= selftest_en ? dds_va : dc_center(ch_data[0], DC_BIAS_CODE);
    pll_chan_r <= selftest_en ? 3'd0   : (pll_vd_d ? pll_ch_d : pll_chan_r);
    pll_val_r  <= selftest_en ? dds_valid : pll_vd_d;
end
wire [11:0] pll_in_data  = pll_data_r;
wire [2:0]  pll_in_chan  = pll_chan_r;
wire        pll_in_valid = pll_val_r;

pll_sync #(
    .CLK_FREQ(CLK_FREQ), .SAMPLES_PER_CYCLE(256), .ADC_BITS(12)
) pll_sync_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .adc_data(pll_in_data), .adc_channel(pll_in_chan), .adc_valid(pll_in_valid),
    .sample_pulse(sample_pulse), .freq_out(freq_out), .phase_out(phase_out),
    .locked(pll_locked_sync), .enable(1'b1)
);

reg [15:0] dds_samp_cnt;
wire dds_samp_pulse;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) dds_samp_cnt <= 16'd0;
    else if (dds_samp_cnt >= 16'd7812) dds_samp_cnt <= 16'd0;
    else dds_samp_cnt <= dds_samp_cnt + 1'b1;
end
assign dds_samp_pulse = (dds_samp_cnt == 16'd7812);

wire eff_sample_pulse = selftest_en ? dds_samp_pulse : sample_pulse;

// --- ADC 栅格触发突发采样（V35，镜像 top.v）：每个采样栅格脉冲触发一轮 6 帧突发 ---
// 机理（V32 栅格遥测上板实锤过零台阶）：ADC 自由跑帧率与 PLL 采样栅格异步拍频
// ——栅格取数的数据年龄周期滑动=相位调制（波形锯齿/过零台阶）。突发同步后每
// 通道数据年龄恒定=纯时延零畸变；A 相帧率被栅格锁定（12.8kHz 相干）。
// V28 帧时序：8.68µs/帧 ×6 = 52.1µs < 最小栅格 71µs（55Hz 钳位），余量 19µs
reg [2:0] adc_burst_cnt;
reg       adc_burst_en_r;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        adc_burst_cnt  <= 3'd0;
        adc_burst_en_r <= 1'b0;
    end else if (eff_sample_pulse) begin
        adc_burst_cnt  <= 3'd0;
        adc_burst_en_r <= 1'b1;               // 栅格脉冲启动新一轮
    end else if (adc_valid && adc_channel < 3'd6) begin
        if (adc_burst_cnt >= 3'd5)
            adc_burst_en_r <= 1'b0;           // 第 6 帧出数后收工，等下一栅格
        else
            adc_burst_cnt <= adc_burst_cnt + 1'b1;
    end
end
assign adc_run_en = adc_burst_en_r;


wire [31:0] fft_mag;
wire [15:0] fft_phase;
wire [7:0]  fft_bin;
wire        fft_done;
wire [2:0]  fft_channel;
wire        fft_mag_valid;

reg [2:0] fft_chan_sel;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) fft_chan_sel <= 3'd0;
    else if (fft_done) fft_chan_sel <= (fft_chan_sel == 3'd5) ? 3'd0 : fft_chan_sel + 3'd1;
end

wire [11:0] dds_fft_v = (fft_chan_sel == 3'd0) ? dds_va :
                        (fft_chan_sel == 3'd1) ? dds_vb : dds_vc;
wire [11:0] dds_fft_i = (fft_chan_sel == 3'd3) ? dds_ia :
                        (fft_chan_sel == 3'd4) ? dds_ib : dds_ic;
wire [11:0] fft_src = selftest_en ? ((fft_chan_sel < 3'd3) ? dds_fft_v : dds_fft_i)
                             : ch_data[fft_chan_sel];

reg [11:0] fft_data_reg;
reg        fft_valid_reg;
always @(posedge clk_100m or negedge rst_n) begin
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
) fft_top_inst (
    .clk(clk_100m), .rst_n(rst_n),
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
wire [15*16-1:0] harm_pct_va;
wire [15*16-1:0] harm_pct_vb;
wire [15*16-1:0] harm_pct_vc;
wire [15*16-1:0] harm_pct_ia;
wire [15*16-1:0] harm_pct_ib;
wire [15*16-1:0] harm_pct_ic;
wire [15:0] phase_ab, phase_bc, phase_ca;
wire [15:0] phase_ia, phase_ib, phase_ic;
wire        phase_sequence;
wire [31:0] unbalance_degree;
wire [177:0] pwr_bus;
wire        measure_done;
wire        done_toggle;

measurement #(
    .FFT_SIZE(256), .HARMONICS_NUM(15)
) measurement_inst (
    .clk(clk_100m), .rst_n(rst_n),
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

wire [2:0]  event_alarm;
wire [31:0] event_count;

event_detect event_detect_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .rms_va(rms_va), .rms_vb(rms_vb), .rms_vc(rms_vc),
    .measure_done(measure_done),
    .nominal_voltage(32'd138),  .sag_threshold(8'd115),
    .swell_threshold(8'd141), .interrupt_threshold(8'd13),
    .event_alarm(event_alarm), .event_count(event_count),
    .enable(pll_locked_sync)
);

user_input #(
    .CLK_FREQ(CLK_FREQ), .DEBOUNCE_MS(1), .EXT_BTN_ACTIVE_LOW(1)
) user_input_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .btn_func(BTN_FUNC), .btn_view(BTN_VIEW), .btn_freeze(BTN_FREEZE),
    .btn_sel1(BTN_SEL1), .btn_sel2(BTN_SEL2), .btn_sel3(BTN_SEL3),
    .btn_sw2(BTN_SW2), .btn_sw3(BTN_SW3),
    .enc1_a(ENC1_CHA), .enc1_b(ENC1_CHB), .enc1_sw(ENC1_SW),
    .enc2_a(ENC2_CHA), .enc2_b(ENC2_CHB),
    .enc3_a(ENC3_CHA), .enc3_b(ENC3_CHB), .enc3_sw(ENC3_SW),
    .enc4_a(ENC4_CHA), .enc4_b(ENC4_CHB), .enc4_sw(ENC4_SW),
    .func_mode(func_mode), .disp_mode(disp_mode), .freeze(freeze),
    .self_sel(self_sel), .meas_chan(meas_chan), .meas_vi(meas_vi), .meas_pick(meas_pick),
    .func_data(func_data), .func_toggle(func_toggle),
    .view_data(view_data), .view_toggle(view_toggle),
    .mode_change_pulse(mode_change_pulse),
    .ui_event(ui_event)
);

ws2812 #(
    .NLED(10), .CLK_HZ(CLK_FREQ)
) ws2812_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .func_mode(func_mode), .disp_mode(disp_mode), .freeze(freeze),
    .self_sel(self_sel), .meas_chan(meas_chan),
    .meas_vi(meas_vi), .meas_pick(meas_pick),
    .ui_event(ui_event), .event_alarm(event_alarm),
    .din(WS2812_DIN)
);

reg mode_toggle;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) mode_toggle <= 1'b0;
    else if (mode_change_pulse) mode_toggle <= ~mode_toggle;
end

reg        wf_toggle;
reg [1:0]  wf_tg_dly;
reg [11:0] wf_va_r, wf_vb_r, wf_vc_r;
reg [11:0] wf_ia_r, wf_ib_r, wf_ic_r;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        wf_toggle <= 1'b0;
        wf_tg_dly <= 2'd0;
        wf_va_r <= 12'd0; wf_vb_r <= 12'd0; wf_vc_r <= 12'd0;
        wf_ia_r <= 12'd0; wf_ib_r <= 12'd0; wf_ic_r <= 12'd0;
    end else begin
        wf_tg_dly <= {wf_tg_dly[0], eff_sample_pulse};
        if (eff_sample_pulse) begin
            wf_va_r <= selftest_en ? dds_va : dc_center(ch_data[0], DC_BIAS_CODE);
            wf_vb_r <= selftest_en ? dds_vb : dc_center(ch_data[1], DC_BIAS_CODE);
            wf_vc_r <= selftest_en ? dds_vc : dc_center(ch_data[2], DC_BIAS_CODE);
            wf_ia_r <= selftest_en ? dds_ia : dc_center(ch_data[3], DC_BIAS_CODE);
            wf_ib_r <= selftest_en ? dds_ib : dc_center(ch_data[4], DC_BIAS_CODE);
            wf_ic_r <= selftest_en ? dds_ic : dc_center(ch_data[5], DC_BIAS_CODE);
        end
        if (wf_tg_dly[1]) wf_toggle <= ~wf_toggle;
    end
end

reg rst_n_meta, rst_n_px;
always @(posedge clk_74m or negedge rst_n) begin
    if (!rst_n) begin
        rst_n_meta <= 1'b0;
        rst_n_px   <= 1'b0;
    end else begin
        rst_n_meta <= 1'b1;
        rst_n_px   <= rst_n_meta;
    end
end

display_ctrl #(
    .H_RES(1280), .V_RES(720), .DATA_BITS(12), .HARMONICS_NUM(15)
) display_ctrl_inst (
    .clk(clk_74m), .rst_n(rst_n_px),
    .vid_hs(vid_hs), .vid_vs(vid_vs), .vid_de(vid_de),
    .rgb_r(rgb_r), .rgb_g(rgb_g), .rgb_b(rgb_b),
    .rms_va(rms_va), .rms_vb(rms_vb), .rms_vc(rms_vc),
    .rms_ia(rms_ia), .rms_ib(rms_ib), .rms_ic(rms_ic),
    .thd_va(thd_va), .thd_vb(thd_vb), .thd_vc(thd_vc),
    .thd_ia(thd_ia), .thd_ib(thd_ib), .thd_ic(thd_ic),
    .fundamental_freq(freq_out),
    .harm_pct_va(harm_pct_va), .harm_pct_vb(harm_pct_vb), .harm_pct_vc(harm_pct_vc),
    .harm_pct_ia(harm_pct_ia), .harm_pct_ib(harm_pct_ib), .harm_pct_ic(harm_pct_ic),
    .phase_ab(phase_ab), .phase_bc(phase_bc), .phase_ca(phase_ca),
    .phase_ia(phase_ia), .phase_ib(phase_ib), .phase_ic(phase_ic),
    .phase_sequence(phase_sequence), .unbalance_degree(unbalance_degree),
    .done_toggle(done_toggle),
    .wf_toggle(wf_toggle),
    .wf_va(wf_va_r), .wf_vb(wf_vb_r), .wf_vc(wf_vc_r),
    .wf_ia(wf_ia_r), .wf_ib(wf_ib_r), .wf_ic(wf_ic_r),
    .mode_toggle(mode_toggle),
    .freeze(freeze),
    .event_alarm(event_alarm),
    .func_toggle(func_toggle),
    .func_data(func_data),
    .view_toggle(view_toggle),
    .view_data(view_data),
    .pwr_bus(pwr_bus),
    .enable(1'b1)
);

reg [21:0] buz_cnt;
reg [13:0] buz_pwm_cnt;
reg        buz_pwm;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        buz_cnt <= 22'd0; buz_pwm_cnt <= 14'd0; buz_pwm <= 1'b0;
    end else begin
        if (|ui_event[5:0]) buz_cnt <= 22'd2_000_000;
        else if (buz_cnt != 22'd0) buz_cnt <= buz_cnt - 1'b1;

        if (buz_pwm_cnt >= 14'd12_499) begin
            buz_pwm_cnt <= 14'd0;
            buz_pwm     <= ~buz_pwm;
        end else begin
            buz_pwm_cnt <= buz_pwm_cnt + 14'b1;
        end
    end
end
assign BUZZER = (buz_cnt != 22'd0) & buz_pwm;

assign LED[0] = event_alarm[0];
assign LED[1] = event_alarm[1];
assign LED[2] = event_alarm[2];
assign LED[3] = |thd_va[7:0];
assign LED[4] = measure_done;
assign LED[5] = pll_locked_sync;
assign LED[6] = selftest_en;
assign LED[7] = rst_n;

endmodule
