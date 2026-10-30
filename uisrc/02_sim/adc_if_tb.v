`timescale 1ns / 1ps
// adc_if 逐位协议测试：AD7928 行为模型严格按数据手册（Rev. E）建模
//
//   - DIN 在 SCLK 下降沿锁存（t9 建立 10ns / t10 保持 5ns，Table 4）；
//     每帧前 12 个下降沿装载控制寄存器，第 1 个锁存位 = WRITE(bit11)。
//     回归目标：MOSI 若在上升沿事件后立即送下一位（旧实现），WRITE 位
//     从未出现在任何锁存沿上 → 控制寄存器从未写入 → ADC 停在上电默认
//     通道，只有 U 相有数据（2026-10-13 实机故障复现）
//   - DOUT 帧格式（Figure 1/29）：CS↓ + t3(30ns) 后出前导 0；其后每个
//     下降沿 + t4(40ns max) 移出下一位：ADD2..ADD0 + 12bit 数据（MSB 先）；
//     第 16 个下降沿 + t8(20ns) 后 DOUT 三态
//   - 上电控制寄存器建模为 12'h000（ADD=000：通道 0）；帧 N 的控制字
//     选通帧 N+1 的转换通道
//   - 每帧转换码 = f(帧号, 通道) 唯一伪随机，DUT 输出必须逐位一致
//
// 注意：DUT 必须用 SPI_CLK_FREQ=12_500_000（top.v 实际取值）。若设 20MHz
// （SCLK 16.7MHz、半周期 30ns），上升沿采样点距上一下降沿仅 40ns = t4 max，
// 零裕量——本模型的 t4=40ns 延迟会让采样撞上数据翻转，数据比对必然失败。
module adc_if_tb;

reg         clk;
reg         rst_n;
reg         enable;

wire        adc_sclk;
wire        adc_cs_n;
wire        adc_mosi;
wire [11:0] adc_data;
wire [2:0]  adc_channel;
wire        adc_valid;

parameter PERIOD = 10;                    // 100MHz

reg         adc_dout;                     // 模型 DOUT（先于例化声明，避免隐式 wire）

adc_if #(
    .CLK_FREQ     (100_000_000),
    .SPI_CLK_FREQ (12_500_000),           // top.v 实际取值：SCLK 12.5MHz
    .ADC_BITS     (12),
    .CHANNELS     (6)
) uut (
    .clk(clk), .rst_n(rst_n),
    .adc_sclk(adc_sclk), .adc_cs_n(adc_cs_n),
    .adc_miso(adc_dout), .adc_mosi(adc_mosi),
    .adc_data(adc_data), .adc_channel(adc_channel), .adc_valid(adc_valid),
    .enable(enable)
);

always #(PERIOD/2) clk = ~clk;

integer errors;

// ===========================================================================
// AD7928 行为模型
// ===========================================================================
function [11:0] adc_code;                // 帧转换码：随 (帧号, 通道) 唯一
    input integer fidx;
    input [2:0]  ch;
    integer v;
    begin
        v = (fidx * 2833 + ch * 1957 + 3162) % 4096;
        adc_code = v[11:0];
    end
endfunction

reg  [11:0] ctrl_reg;                     // 控制寄存器（上电 0：通道 0）
reg  [2:0]  conv_chan;                    // 本帧转换通道（上一帧 ADD 选通）
reg  [15:0] dout_frame;                   // {前导0, ADD2..0, 12bit 数据}
integer     dout_bit;                     // 下一个移出的位下标（15 先出）
integer     fall_cnt, rise_cnt;
integer     frame_idx;
integer     ctrl_wr_cnt;                  // 控制寄存器成功写入次数（>0 = 接口通）
reg  [11:0] din_shift;
real        din_tchange;                  // DIN 最近一次变化时刻
real        fall_time;                    // 最近一次下降沿时刻
real        fall16_time;                  // 本帧第 16 个下降沿时刻
real        cs_fall_time, cs_rise_time;
real        rise1_time;                   // 本帧第一个上升沿时刻
reg  [11:0] frame_val;                    // 本帧转换码（供 DUT 输出比对）
reg  [2:0]  exp_tag;                      // 刚完成帧的通道（CS↑ 锁存，供 valid 比对）
integer     dut_emitted;

always @(adc_mosi) begin
    din_tchange = $realtime;
    // t10：DIN 不得在下降沿后 5ns 内变化
    if (fall_time > 0 && ($realtime - fall_time) < 4.0) begin
        $display("[FAIL %0t] t10 violation: DIN changed %0f ns after SCLK falling #%0d",
                 $time, $realtime - fall_time, fall_cnt);
        errors = errors + 1;
    end
end

// DIN 在下降沿锁存：前 12 位进控制移位器，第 12 位完成时装载
always @(negedge adc_sclk) begin
    if (!adc_cs_n) begin
        fall_time = $realtime;
        fall_cnt  = fall_cnt + 1;
        // t9：DIN 建立时间 ≥ 10ns（容 1ns 仿真量子）
        if (din_tchange > 0 && (fall_time - din_tchange) < 9.0) begin
            $display("[FAIL %0t] t9 violation: DIN setup %0f ns < 10ns (falling #%0d)",
                     $time, fall_time - din_tchange, fall_cnt);
            errors = errors + 1;
        end
        if (fall_cnt == 16) fall16_time = fall_time;
        if (fall_cnt <= 12) begin
            din_shift = {din_shift[10:0], adc_mosi};
            if (fall_cnt == 12) begin
                if (din_shift[11] === 1'b1) begin
                    ctrl_reg    = din_shift;
                    ctrl_wr_cnt = ctrl_wr_cnt + 1;
                end else begin
                    $display("[FAIL %0t] frame %0d: WRITE bit not 1 at control bit11 (DIN misaligned?)",
                             $time, frame_idx);
                    errors = errors + 1;
                end
            end
        end
        // DOUT：下降沿 #1..#15 各移出下一位（t4=40ns max 后有效）
        if (fall_cnt <= 15) begin
            #40 adc_dout = dout_frame[dout_bit];
            dout_bit = dout_bit - 1;
        end else begin
            #20 adc_dout = 1'bz;          // 第 16 个下降沿：t8 后三态
        end
    end
end

always @(posedge adc_sclk) begin
    if (!adc_cs_n) begin
        rise_cnt = rise_cnt + 1;
        if (rise_cnt == 1) rise1_time = $realtime;
    end
end

// 帧组装：CS↓ 出前导 0（t3=30ns 后），记录 tQUIET 起点检查上一帧间隔
always @(negedge adc_cs_n) if (rst_n) begin
    cs_fall_time = $realtime;
    if (cs_rise_time > 0 && (cs_fall_time - cs_rise_time) < 45.0) begin
        $display("[FAIL %0t] tQUIET violation: CS high window %0f ns (< 50ns)",
                 $time, cs_fall_time - cs_rise_time);
        errors = errors + 1;
    end
    fall_cnt  = 0;
    rise_cnt  = 0;
    dout_bit  = 14;
    adc_dout  = 1'bz;
    frame_val = adc_code(frame_idx, conv_chan);
    dout_frame = {1'b0, conv_chan, frame_val};
    dut_emitted = 0;
    #30 adc_dout = dout_frame[15];       // t3 = 30ns max 出前导 0
end

// CS↑：帧结束（沿数 / t11 检查；本帧控制字选通下一帧通道）
always @(posedge adc_cs_n) if (rst_n) begin
    cs_rise_time = $realtime;
    if (fall_cnt != 16) begin
        $display("[FAIL %0t] frame %0d: %0d SCLK falling edges in frame (expect 16)",
                 $time, frame_idx, fall_cnt);
        errors = errors + 1;
    end
    if (rise_cnt != 16) begin
        $display("[FAIL %0t] frame %0d: %0d SCLK rising edges in frame (expect 16)",
                 $time, frame_idx, rise_cnt);
        errors = errors + 1;
    end
    if (fall16_time > 0 && (cs_rise_time - fall16_time) < 19.0) begin
        $display("[FAIL %0t] t11 violation: CS risen %0f ns after 16th falling (< 20ns)",
                 $time, cs_rise_time - fall16_time);
        errors = errors + 1;
    end
    // 第一个上升沿须在 CS↓ 后 ≥35ns（t2=10ns + t3=30ns，保证首拍能采到前导 0）
    if (rise1_time > 0 && (rise1_time - cs_fall_time) < 35.0) begin
        $display("[FAIL %0t] first SCLK rising only %0f ns after CS falling (< 35ns)",
                 $time, rise1_time - cs_fall_time);
        errors = errors + 1;
    end
    adc_dout = 1'bz;
    exp_tag   = conv_chan;               // 本帧刚转换完的通道
    conv_chan = ctrl_reg[8:6];           // 本帧控制字选通下一帧通道
    frame_idx = frame_idx + 1;
end

// ===========================================================================
// DUT 输出检查：每帧恰一次 valid，通道序与数据逐一比对
// ===========================================================================
integer seq_ok, seq_n;
always @(posedge clk) begin
    if (adc_valid) begin
        if (dut_emitted) begin
            $display("[FAIL %0t] multiple adc_valid in frame %0d", $time, frame_idx);
            errors = errors + 1;
        end
        dut_emitted = 1;
        if (adc_channel >= 3'd6) begin
            $display("[FAIL %0t] accepted out-of-range channel %0d", $time, adc_channel);
            errors = errors + 1;
        end else begin
            if (frame_idx >= 1) begin
                if (adc_channel !== exp_tag) begin
                    $display("[FAIL %0t] frame %0d tag=%0d expect %0d (channel stuck = power-up default?)",
                             $time, frame_idx, adc_channel, exp_tag);
                    errors = errors + 1;
                end else begin
                    seq_ok = seq_ok + 1;
                end
                seq_n = seq_n + 1;
            end
            if (adc_data !== frame_val) begin
                $display("[FAIL %0t] frame %0d ch%0d data=%03h expect %03h (bit slip?)",
                         $time, frame_idx, adc_channel, adc_data, frame_val);
                errors = errors + 1;
            end
        end
    end
end

initial begin
    clk = 1'b0; rst_n = 1'b0; enable = 1'b0;
    errors = 0; seq_ok = 0; seq_n = 0;
    ctrl_reg = 12'h000;                  // 上电默认：通道 0（未初始化写前恒转 VIN0）
    conv_chan = 3'd0;
    adc_dout = 1'bz;
    frame_idx = 0; frame_val = 12'd0; exp_tag = 3'd0;
    dout_frame = 16'd0; dout_bit = 14;
    fall_cnt = 0; rise_cnt = 0; ctrl_wr_cnt = 0; dut_emitted = 0;
    din_shift = 12'd0;
    din_tchange = 0.0; fall_time = 0.0; fall16_time = 0.0;
    cs_fall_time = 0.0; cs_rise_time = 0.0; rise1_time = 0.0;
    #1000;
    rst_n  = 1'b1;
    enable = 1'b1;
end

// ===========================================================================
// 主流程
// ===========================================================================
initial begin
    #60_000;                             // ≈ 35 帧
    $display("--- Test 1: control register written every frame ---");
    if (ctrl_wr_cnt >= 30)
        $display("PASS: %0d control writes in %0d frames", ctrl_wr_cnt, frame_idx);
    else begin
        $display("FAIL: only %0d control writes in %0d frames (WRITE bit never latched)",
                 ctrl_wr_cnt, frame_idx);
        errors = errors + 1;
    end

    $display("--- Test 2: channel round-robin + data integrity ---");
    if (seq_ok >= 20 && seq_n >= 20 && errors == 0)
        $display("PASS: %0d/%0d frames tag/data correct", seq_ok, seq_n);
    else begin
        $display("FAIL: seq_ok=%0d seq_n=%0d errors=%0d", seq_ok, seq_n, errors);
        errors = errors + 1;
    end

    $display("--- Test 3: disable stops conversion ---");
    enable = 1'b0;
    #20_000;
    if (uut.state == 4'd0)
        $display("PASS: FSM returns to IDLE on disable");
    else begin
        $display("FAIL: state=%0d after disable", uut.state);
        errors = errors + 1;
    end

    enable = 1'b1;
    #20_000;
    if (frame_idx > 40)
        $display("PASS: conversions resume after re-enable (%0d frames)", frame_idx);
    else begin
        $display("FAIL: frames=%0d", frame_idx);
        errors = errors + 1;
    end

    if (errors == 0) $display("=== adc_if_tb ALL PASS ===");
    else             $display("=== adc_if_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #1_000_000;
    $display("TIMEOUT: adc_if_tb (errors=%0d, ctrl_wr=%0d, frames=%0d)",
             errors, ctrl_wr_cnt, frame_idx);
    $finish;
end

endmodule
