module top #(
    parameter CLK_FREQ = 100_000_000
)(
    input  wire        SYSCLK_25M,

    output wire        ADC_SCLK,
    output wire        ADC_CS_N,
    input  wire        ADC_MISO,
    output wire        ADC_MOSI,

    // HDMI TMDS 差分输出：LVDS33_E 由 TD 按 P 脚自动配对驱动 N 脚
    // 通道映射（与板卡硬件一致）：TX[0]=蓝=P47，TX[1]=绿=P49，TX[2]=红=P61，CLK=P55
    output wire [2:0]  HDMI_TX_P,
    output wire        HDMI_CLK_P,

    output wire [7:0]  LED,

    // 按键板（J4 引出，低有效，外接上拉；详见 top.adc 注释）
    input  wire        BTN_FUNC,      // 功能：自检/单相A/三相四线/三相三线
    input  wire        BTN_VIEW,      // 显示：示波/频谱/谐波表
    input  wire        BTN_FREEZE,    // RUN/STOP 定格
    input  wire        BTN_SEL1,      // 情境键1（自检:谐波 / 测量:通道循环）
    input  wire        BTN_SEL2,      // 情境键2（自检:不平衡 / 测量:V/I 切换）
    input  wire        BTN_SEL3,      // 情境键3（自检:逆相序 / 测量:选中波形）
    input  wire        BTN_SW2,       // 核心板备份键 = VIEW
    input  wire        BTN_SW3,       // 核心板备份键 = FREEZE

    // 按键板 4 个增量编码器（正交 A/B 通道；1/3/4 带按压复位）
    input  wire        ENC1_CHA, ENC1_CHB,   // 时基
    input  wire        ENC1_SW,              // 时基按压复位
    input  wire        ENC2_CHA, ENC2_CHB,   // 平移
    input  wire        ENC3_CHA, ENC3_CHB,   // 垂直位置
    input  wire        ENC3_SW,              // 垂直位置按压复位
    input  wire        ENC4_CHA, ENC4_CHB,   // 垂直缩放
    input  wire        ENC4_SW,              // 垂直缩放按压复位

    output wire        WS2812_DIN,    // 按键板 RGB 灯链

    output wire        BUZZER,

    output wire        UART_TXD,
    input  wire        UART_RXD,

    inout  wire        I2C_SDA,
    output wire        I2C_SCL
);

wire        clk_100m;
wire        clk_74m;
wire        clk_5x;
wire        pll_sys_locked;
wire        pll_hdmi_locked;
wire        pll_all_locked;
wire        rst_n;

assign pll_all_locked = pll_sys_locked & pll_hdmi_locked;

reg [7:0]  rst_counter;
always @(posedge clk_100m or negedge pll_all_locked) begin
    if (!pll_all_locked) rst_counter <= 8'd0;
    else if (rst_counter < 8'd255) rst_counter <= rst_counter + 1'b1;
end
assign rst_n = (rst_counter >= 8'd200);

// EG4 PLL 需复位脉冲才能可靠锁定：配置完成后延迟约 1.3ms 释放系统 PLL 复位（高有效）
reg [15:0] pll_rst_cnt = 16'd0;
always @(posedge SYSCLK_25M) begin
    if (pll_rst_cnt[15]) pll_rst_cnt <= pll_rst_cnt;
    else pll_rst_cnt <= pll_rst_cnt + 1'b1;
end

pll_sys pll_sys_inst (
    .refclk   (SYSCLK_25M),
    .reset    (!pll_rst_cnt[15]),
    .extlock  (pll_sys_locked),
    .clk0_out (clk_100m)
);

wire clk_hdmi_100m;
// HDMI PLL 保持复位直到系统 PLL 锁定后释放（厂商 07 双 PLL 例程同款顺序）
pll_hdmi pll_hdmi_inst (
    .refclk   (SYSCLK_25M),
    .reset    (!pll_sys_locked),
    .extlock  (pll_hdmi_locked),
    .clk0_out (clk_hdmi_100m),
    .clk1_out (clk_74m),
    .clk2_out (clk_5x)
);

wire adc_run_en;
wire [11:0] adc_data;
wire [2:0]  adc_channel;
wire        adc_valid;

adc_if #(
    // SCLK=12.5MHz：DOUT 采样点距上一下降沿 50ns，较 t4(40ns max) 留 10ns 裕量；
    // 20MHz 档(16.7MHz)该裕量归零，实机逐位采样会撞上 DOUT 翻转
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
wire [1:0]  meas_chan;
wire [1:0]  meas_vi;       // 0=双显(V+I) 1=仅电压 2=仅电流
wire [1:0]  meas_pick;
wire [10:0] func_data;
wire        func_toggle;
wire [44:0] view_data;
wire        view_toggle;
wire        mode_change_pulse;
wire [9:0]  ui_event;         // 按键/编码器操作事件（WS2812 灯效）

// 上电默认 func_mode=0（自检）；板载拨码已从逻辑与约束中删除
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

// PLL 输入选择：自检模式锁 DDS-A（频率读数真实），ADC 模式锁 A 相采样通道
// PLL 输入：A相经直流校正（减采样板偏置）——小信号叠加偏置时原始码可能整体
// 高于/低于 2048 而永无过零，锁相失效（频率乱跳、采样失步、波形畸变）。
// 校正结果寄存一拍（valid/chan 同步延迟对齐），切断 adc_data→dc_center→锁相
// 比较器的跨模块组合长路（曾致 100MHz 域时序劣化）
reg [11:0] pll_data_r;
reg [2:0]  pll_chan_r;
reg        pll_val_r;
// V28：改读中值滤波后的 ch_data[0]——中值回写在 adc_valid 后第 2 拍生效
//（拍1 锁存帧、拍2 写中值），valid/chan 同步延迟同拍配对（pll_val_r 置位拍
// ch_data[0] 必为最新中值输出）
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

// --- ADC 栅格触发突发采样（V35）：每个采样栅格脉冲触发一轮 6 帧突发 ---
// 机理（V32 栅格遥测上板实锤过零台阶 / 用户实测波形不丝滑）：ADC 自由跑帧率
// 与 PLL 采样栅格异步拍频——栅格取数的数据年龄在 0~帧周期内周期性滑动=对信号
// 做相位调制（时域=过零台阶/波形锯齿，频域=FFT 泄漏抬 THD，读数=RMS 逐窗摆）。
// 突发同步后每轮恰好覆盖 0..5 全部通道一次（DOUT 回读标记天然对齐），每通道
// 数据年龄恒定=纯时延零畸变；A 相帧率被栅格锁定（256×工频，50Hz 时 12.8kHz，
// 与栅格完全相干）。
// 帧时序预算：8.68µs/帧 ×6 = 52.1µs < 最小栅格 71µs（55Hz 钳位），余量 19µs；
// 通道轮转继承式（每轮第 1 帧转换上一轮末控制字所选通道，6 帧报告值=N-1..N+4
// mod 6 恰好遍历全部通道），burst_cnt 只数有效帧数、与通道次序无关。
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

// --- FFT 通道轮换：每完成一个通道频谱换下一个（0=Va→1=Vb→2=Vc→3=Ia→4=Ib→5=Ic）---
reg [2:0] fft_chan_sel;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) fft_chan_sel <= 3'd0;
    else if (fft_done) fft_chan_sel <= (fft_chan_sel == 3'd5) ? 3'd0 : fft_chan_sel + 3'd1;
end

// 自检：DDS 6 通道（电压 0-2 / 电流 3-5）；测量：ADC 6 通道
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

// V26 资源减法：64 条事件缓存的读出端口在 top 级全部悬空（无任何消费方），
// 删除不可观察存储；检测/确认/报警/计数行为逐位不变
event_detect event_detect_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .rms_va(rms_va), .rms_vb(rms_vb), .rms_vc(rms_vc),
    .measure_done(measure_done),
    .nominal_voltage(32'd138),  .sag_threshold(8'd115),  // 标称 13.8V（24V 母线相电压）；48V 母线改 277（27.7V）
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

// --- 按键板 RGB 灯链：颜色随功能/显示模式/定格/注入状态联动 ---
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

// --- 模式切换：脉冲→翻转（100MHz→75MHz 像素域用翻转握手，避免漏检）---
reg mode_toggle;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) mode_toggle <= 1'b0;
    else if (mode_change_pulse) mode_toggle <= ~mode_toggle;
end

// --- 波形采样送显示：数据先更新，2 拍后才翻转握手（保证翻转沿时数据已稳定）---
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

wire        vid_hs, vid_vs, vid_de;
wire [7:0]  rgb_r, rgb_g, rgb_b;

// rst_n 释放同步到 75MHz 像素域：异步复位、同步释放
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
    // 频率显示用 pll_sync 实测值（0.1Hz 单位，两种模式均为真读数）；
    // measurement 的 fundamental_freq 在锁相采样下恒为 bin×50 的名义值
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

// 厂商 HDMI 发送 IP：TMDS 8b/10b 编码 + 10:1 LVDS 串行化（EG_LOGIC_ODDR）
uihdmitx #(
    .FAMILY("EG4")
) uihdmitx_inst (
    .RSTn_i(pll_hdmi_locked),
    .HS_i(vid_hs),
    .VS_i(vid_vs),
    .DE_i(vid_de),
    .RGB_i({rgb_r, rgb_g, rgb_b}),
    .PCLKX1_i(clk_74m),
    .PCLKX5_i(clk_5x),
    .HDMI_CLK_P(HDMI_CLK_P),
    .HDMI_TX_P(HDMI_TX_P)
);

// --- 蜂鸣器：按键按下短促提示音（约 20ms；编码器调节不响，避免连续噪声）---
// 板载为无源蜂鸣器，必须用 2~4kHz 方波驱动（直流电平不发声）：
// 使能窗口 20ms 内输出 4kHz 方波（半周期 12500 拍 @100MHz）
reg [21:0] buz_cnt;
reg [13:0] buz_pwm_cnt;
reg        buz_pwm;
always @(posedge clk_100m or negedge rst_n) begin
    if (!rst_n) begin
        buz_cnt <= 22'd0; buz_pwm_cnt <= 14'd0; buz_pwm <= 1'b0;
    end else begin
        if (|ui_event[5:0]) buz_cnt <= 22'd2_000_000;   // 20ms @ 100MHz
        else if (buz_cnt != 22'd0) buz_cnt <= buz_cnt - 1'b1;

        if (buz_pwm_cnt >= 14'd12_499) begin
            buz_pwm_cnt <= 14'd0;
            buz_pwm     <= ~buz_pwm;                     // 4kHz 方波
        end else begin
            buz_pwm_cnt <= buz_pwm_cnt + 14'd1;
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

// --- VOFA+ 遥测（V35）：A 相 SPI 原始帧电压，纯旁路零侵入 ---
// 与 JTAG 共用核心板 TYPE-C 板载 USB 转串口，无需接线。921600/FireWater。
// 每行 = A 相电压毫伏（±20V 输入显示 ±20000），独立定时器保证 1000 行/秒
//（VOFA Δt=1 恒真实时间，与工频无关——突发同步后 A 帧率=栅格率 11.5~14.1kHz
// 随工频漂移，按帧率抽选会使行率跟着漂，故改 100MHz 分频定时器）。
// 数据源 = ADC 原始帧（中值滤波前，SPI 采到什么发什么）；只读 adc_* 总线，
// 测量/显示/锁相路径零改动。DC_BIAS_CODE 与本文件直流常数同源
wire vofa_tx;
vofa_stream #(
    .CLK_FREQ(CLK_FREQ), .BAUD(921600), .LINE_RATE_HZ(1000),
    .DC_BIAS_CODE(DC_BIAS_CODE)
) vofa_stream_inst (
    .clk(clk_100m), .rst_n(rst_n),
    .adc_data(adc_data), .adc_channel(adc_channel), .adc_valid(adc_valid),
    .uart_txd(vofa_tx)
);
assign UART_TXD = vofa_tx;   // 原 V28 恒 1
assign I2C_SCL  = 1'b1;

endmodule
