// AD7928 8 通道 12bit SAR ADC SPI 主机（采样板：VIN0..2=三相电压 AMC1350，VIN3..5=三相电流 CC6905）
//
// 控制寄存器（数据手册 Table 7，12bit，MSB 先发，每帧重写选择"下一帧"的转换通道）：
//   bit11 WRITE=1  bit10 SEQ=0  bit9 x  bit8:6 ADD2..ADD0  bit5:4 PM=11(正常模式)
//   bit3  SHADOW=0 bit2 x       bit1 RANGE=1(0~VREF=2.5V 量程)  bit0 CODING=1(直接二进制)
//
// DOUT 帧（16bit，MSB 先出）：CS 下降沿给出前导 0，随后每个 SCLK 下降沿移出下一位：
//   [15]=前导0  [14:12]=本帧转换结果的通道地址  [11:0]=12bit 转换结果
//   → 通道标记直接取自 DOUT 回读地址，天然规避"控制字选的是下一帧通道"的差一错位
//
// 时序要点（数据手册 Table 4，按 SCLK=12.5MHz/半周期 40ns 核算）：
//   fSCLK≤20MHz，且采样点距上一下降沿须 ≥t4(40ns max)——16.7MHz(20MHz 档)时
//   该裕量归零，故 top.v 实际取 12.5MHz；DIN 在 SCLK 下降沿锁存（t9=10ns/t10=5ns）：
//   WRITE 位自 CS↓ 起上总线覆盖第 1 个下降沿，后续位在每个下降沿事件后 1 拍送出
//   （到下一下降沿有整周期建立时间）；MISO 在 SCLK 上升沿事件后 10ns 采样
//   （距上一下降沿 50ns ≥ t4 40ns）；CS 下降后延迟 4 拍才出第一个 SCLK 沿
//   （t2 建立 + t3 出三态 35ns）；第 16 个下降沿后 SCLK 停摆、CS 保持低 ≥5 拍
//   （t11=20ns），帧间 CS 高 ≥6 拍（tQUIET=50ns）
module adc_if #(
    parameter CLK_FREQ     = 100_000_000,
    parameter SPI_CLK_FREQ = 12_500_000,   // SCLK=CLK/(2*SPI_DIV)，向上取整保证不超 20MHz 规格
    parameter ADC_BITS     = 12,
    parameter CHANNELS     = 6             // 只扫描 VIN0..5，VIN6/7 悬空(接 AGND)
)(
    input  wire        clk,
    input  wire        rst_n,

    output wire        adc_sclk,
    output wire        adc_cs_n,
    input  wire        adc_miso,
    output wire        adc_mosi,

    output wire [ADC_BITS-1:0] adc_data,
    output wire [2:0]         adc_channel,
    output wire               adc_valid,

    input  wire        enable
);

// 向上取整：请求 20MHz 时 SPI_DIV=3(16.7MHz)，请求 12.5MHz 时 SPI_DIV=4(12.5MHz)
localparam SPI_DIV = (CLK_FREQ + 2 * SPI_CLK_FREQ - 1) / (2 * SPI_CLK_FREQ);

reg [7:0] spi_div_cnt;
reg [1:0] cs_gap;
reg       spi_clk_en;
reg       spi_clk_reg;
reg       sclk_hold;                     // 第 16 个下降沿后 SCLK 停摆（帧恒 16 沿）

reg [3:0]  spi_bit_cnt;
reg [15:0] spi_shift_reg;
reg [15:0] rx_data_reg;

reg [2:0] next_channel;

reg [3:0] state;
localparam IDLE      = 4'd0;
localparam START_SEQ = 4'd1;
localparam TRANSFER  = 4'd2;
localparam END_WAIT  = 4'd3;   // 第 16 个 SCLK 下降沿后保持 CS 低（t11）
localparam DONE      = 4'd4;
localparam DELAY     = 4'd5;   // 帧间静默（tQUIET）

reg [ADC_BITS-1:0] output_data;
reg [2:0]  output_channel;
reg        output_valid;

reg [3:0] delay_cnt;

reg        spi_cs_n_reg;
reg        spi_mosi_reg;

// 控制字：WRITE|SEQ=0|x|ADD|PM=11|SHADOW=0|x|RANGE=1|CODING=1
// 16bit 帧左对齐：AD7928 每帧 16 个 SCLK，控制字 MSB 先行占 [15:4]，低 4 位补零。
// 原声明只有 [11:0] 却按 [15]/[14:0] 取用——越界位仿真为 X（破坏真实 ADC 通信），TD 拒绝
wire [15:0] ctrl_word = {1'b1,            // [15] WRITE=1
                         1'b0,            // [14] SEQ=0（单转换，每帧重写通道）
                         1'b0,            // [13] x
                         next_channel,    // [12:10] ADD2..ADD0（下一帧转换通道）
                         2'b11,           // [9:8] PM=11 正常模式
                         1'b0,            // [7] SHADOW=0
                         1'b0,            // [6] x
                         1'b1,            // [5] RANGE=1（0~2.5V，采样板基准）
                         1'b1,            // [4] CODING=1 直接二进制
                         4'b0000};        // [3:0] 帧尾填充

// DOUT 回读帧：[14:12]=通道地址 [11:0]=数据
wire [2:0] rx_chan = rx_data_reg[14:12];

// ---------------------------------------------------------------------------
// SCLK 产生：CS 低后先等 4 拍（t2 + DOUT 脱离三态），再按 SPI_DIV 分频翻转；
// CS 高期间计数器清零、SCLK 保持低（CPOL=0），每帧第一个沿必为上升沿；
// 第 16 个下降沿后 sclk_hold 冻结翻转——END_WAIT 期间不再产生第 17 个沿
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        spi_div_cnt <= 8'd0;
        cs_gap      <= 2'd0;
        spi_clk_en  <= 1'b0;
        spi_clk_reg <= 1'b0;
    end else if (spi_cs_n_reg || sclk_hold) begin
        spi_div_cnt <= 8'd0;
        cs_gap      <= 2'd0;
        spi_clk_en  <= 1'b0;
        spi_clk_reg <= 1'b0;
    end else begin
        if (cs_gap != 2'd3) begin
            cs_gap     <= cs_gap + 1'b1;
            spi_clk_en <= 1'b0;
        end else if (spi_div_cnt >= SPI_DIV[7:0] - 1'b1) begin
            spi_div_cnt <= 8'd0;
            spi_clk_en  <= 1'b1;
            spi_clk_reg <= ~spi_clk_reg;
        end else begin
            spi_div_cnt <= spi_div_cnt + 1'b1;
            spi_clk_en  <= 1'b0;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= IDLE;
        next_channel    <= 3'd0;
        spi_bit_cnt     <= 4'd0;
        spi_shift_reg   <= 16'd0;
        rx_data_reg     <= 16'd0;
        delay_cnt       <= 4'd0;
        spi_cs_n_reg    <= 1'b1;
        spi_mosi_reg    <= 1'b0;
        output_data     <= {ADC_BITS{1'b0}};
        output_channel  <= 3'd0;
        output_valid    <= 1'b0;
        sclk_hold       <= 1'b0;
    end else begin
        output_valid <= 1'b0;

        case (state)
            IDLE: begin
                spi_cs_n_reg <= 1'b1;
                spi_mosi_reg <= 1'b0;
                if (enable) begin
                    state <= START_SEQ;
                end
            end

            // CS 拉低、装载控制字。MOSI 立即输出 bit1(WRITE)——AD7928 在 SCLK
            // 下降沿锁存 DIN，该位须覆盖第 1 个下降沿；后续位在各下降沿事件
            // 后 1 拍送出（旧实现在上升沿事件就送下一位，早于第 1 个下降沿，
            // WRITE 位从未落在任何锁存沿上，控制寄存器从未被写入，ADC 停在
            // 上电默认通道 0——实机表现即"只有 U 相有数据"）
            START_SEQ: begin
                spi_shift_reg <= {ctrl_word[14:0], 1'b0};
                spi_mosi_reg  <= ctrl_word[15];
                spi_cs_n_reg  <= 1'b0;
                sclk_hold     <= 1'b0;
                spi_bit_cnt   <= 4'd0;
                state         <= TRANSFER;
            end

            // 上升沿事件采样 DOUT（MSB 先）；下降沿事件移出下一位 DIN
            TRANSFER: begin
                if (spi_clk_en && spi_clk_reg) begin
                    rx_data_reg <= {rx_data_reg[14:0], adc_miso};
                    if (spi_bit_cnt >= 4'd15) begin
                        state <= END_WAIT;
                    end else begin
                        spi_bit_cnt <= spi_bit_cnt + 1'b1;
                    end
                end else if (spi_clk_en && !spi_clk_reg) begin
                    spi_mosi_reg  <= spi_shift_reg[15];
                    spi_shift_reg <= {spi_shift_reg[14:0], 1'b0};
                end
            end

            // 第 16 个下降沿后停摆 SCLK，再等 4 拍（t11=20ns）抬高 CS
            END_WAIT: begin
                if (!sclk_hold) begin
                    if (spi_clk_en && !spi_clk_reg) sclk_hold <= 1'b1;
                    delay_cnt <= 4'd0;          // 等待第 16 个下降沿期间不计数
                end else if (delay_cnt >= 4'd3) begin
                    delay_cnt <= 4'd0;
                    state     <= DONE;
                end else begin
                    delay_cnt <= delay_cnt + 1'b1;
                end
            end

            // 通道标记取自 DOUT 回读地址：上电首帧控制寄存器未初始化、
            // 结果可能来自任意通道，地址≥CHANNELS（含悬空的 VIN6/7）时丢弃
            DONE: begin
                spi_cs_n_reg <= 1'b1;
                if (rx_chan < CHANNELS[2:0]) begin
                    output_data    <= rx_data_reg[ADC_BITS-1:0];
                    output_channel <= rx_chan;
                    output_valid   <= 1'b1;
                end
                if (next_channel >= CHANNELS[2:0] - 3'd1) begin
                    next_channel <= 3'd0;
                end else begin
                    next_channel <= next_channel + 3'd1;
                end
                delay_cnt <= 4'd6;
                state     <= DELAY;
            end

            // CS 高静默 ≥50ns 后进入下一帧；enable 撤销则回 IDLE（扫完当前帧再停）
            DELAY: begin
                if (delay_cnt > 4'd0) begin
                    delay_cnt <= delay_cnt - 1'b1;
                end else begin
                    state <= enable ? START_SEQ : IDLE;
                end
            end

            default: state <= IDLE;
        endcase
    end
end

assign adc_sclk    = spi_clk_reg;
assign adc_cs_n    = spi_cs_n_reg;
assign adc_mosi    = spi_mosi_reg;
assign adc_data    = output_data;
assign adc_channel = output_channel;
assign adc_valid   = output_valid;

endmodule
