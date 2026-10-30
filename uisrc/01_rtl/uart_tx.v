// UART 发送器（8N1，平凡写法：移位+计数，无函数/无符号运算）
// 用法：wr 置一拍（仅在 !busy 时），din 同拍有效；busy 拉高期间数据移出
module uart_tx #(
    parameter CLK_FREQ = 100_000_000,
    parameter BAUD     = 921600
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] din,
    input  wire       wr,
    output reg        tx,
    output wire       busy
);
    localparam DIV = CLK_FREQ / BAUD;          // 100MHz/921600 = 108 拍/位（+0.5% 误差，UART 容差 2%+）
    localparam CNT_W = (DIV < 2) ? 1 :
                       (DIV < 4) ? 2 :
                       (DIV < 8) ? 3 :
                       (DIV < 16) ? 4 :
                       (DIV < 32) ? 5 :
                       (DIV < 64) ? 6 :
                       (DIV < 128) ? 7 :
                       (DIV < 256) ? 8 :
                       (DIV < 512) ? 9 :
                       (DIV < 1024) ? 10 :
                       (DIV < 2048) ? 11 : 12;

    reg [CNT_W-1:0] div_cnt;
    reg [3:0]  bit_idx;                        // 0=起始位 1..8=数据 9=停止位 10=IDLE
    reg [7:0]  sh;

    assign busy = (bit_idx != 4'd10);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
            bit_idx <= 4'd10;
            sh      <= 8'd0;
            tx      <= 1'b1;
        end else begin
            if (bit_idx == 4'd10) begin
                tx <= 1'b1;
                if (wr) begin
                    sh      <= din;
                    bit_idx <= 4'd0;           // 起始位
                    div_cnt <= 0;
                    tx      <= 1'b0;
                end
            end else if (div_cnt >= DIV - 1) begin
                div_cnt <= 0;
                if (bit_idx == 4'd9) begin
                    bit_idx <= 4'd10;          // 停止位发完回 IDLE（tx 保持 1）
                    tx      <= 1'b1;
                end else begin
                    // bit_idx=当前正在发的位（0=起始位）：到期后发下一位
                    // 下一位序号=bit_idx+1：1..8 为数据位 bit0..7 → sh[bit_idx]
                    // 序号 9=停止位 → 1
                    if (bit_idx == 4'd8)
                        tx <= 1'b1;
                    else
                        tx <= sh[bit_idx[2:0]];
                    bit_idx <= bit_idx + 1'b1;
                end
            end else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end
endmodule
