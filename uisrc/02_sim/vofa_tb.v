// vofa_stream V35 验证：A 相 SPI 原始帧 → 毫伏行
//  1) 原始码 1500（偏置 2160）→ |660|×24.4140625 = -16113 mV 精确断言
//  2) 切原始码 2600 → +440×24.4140625 = +10742 mV 精确断言（更新生效）
// A 帧桩：5208 拍周期（时钟对齐——#延时桩与 clk 同刻 DUT 看不见）
// V35：行率由 100MHz 独立定时器保证（1000 行/秒），与 A 帧率解耦——
// 10ms 窗口恒 ~10 行（定时器节拍），帧桩速率只影响数据新鲜度
// TB 加速：BAUD=2M（DIV=50 拍/位）；毫伏期望与真实波特率下逐位一致
`timescale 1ns / 1ps
module vofa_tb;

reg clk = 0;
reg rst_n;
always #5 clk = ~clk;                    // 100MHz

reg [11:0] adc_data;
reg [2:0]  adc_channel;
reg        adc_valid;
wire       uart_txd;

vofa_stream #(
    .CLK_FREQ(100_000_000), .BAUD(2_000_000),
    .LINE_RATE_HZ(1000)
) dut (
    .clk(clk), .rst_n(rst_n),
    .adc_data(adc_data), .adc_channel(adc_channel), .adc_valid(adc_valid),
    .uart_txd(uart_txd)
);

integer errors;
integer lines_seen;
integer rxd_ok;
integer rxd_badstop;
integer grace;                           // 值切换后在途行宽限
initial begin
    errors = 0; lines_seen = 0; rxd_ok = 0; rxd_badstop = 0; grace = 0;
end

// ============================================================
// A 相帧桩：5208 拍周期，单拍 valid，channel=0
// ============================================================
reg [12:0] fcnt = 0;
always @(posedge clk) begin
    fcnt      <= (fcnt == 13'd5207) ? 13'd0 : fcnt + 1'b1;
    adc_valid <= (fcnt == 13'd5207);
end
initial begin
    adc_channel = 3'd0;
    adc_data    = 12'd1500;
end

// ============================================================
// 时钟化接收状态机（1x 中点采样）：字节→行→解析
// ============================================================
localparam DIV = 50;                     // 拍/位（BAUD=2M @100MHz）
reg [1:0] rstate;
reg [6:0] rcnt;
reg [2:0] rbit;
reg [7:0] rshr;
reg [7:0] line_buf [0:63];
integer   line_chars;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rstate <= 2'd0; rcnt <= 7'd0; rbit <= 3'd0; rshr <= 8'd0;
        line_chars = 0;
    end else begin
        case (rstate)
            2'd0: if (!uart_txd) begin rstate <= 2'd1; rcnt <= 7'd0; end
            2'd1: begin
                rcnt <= rcnt + 1'b1;
                if (rcnt >= DIV/2 + DIV - 1) begin           // 1.5 位处：bit0 中点
                    rshr <= {uart_txd, rshr[7:1]};
                    rbit <= 3'd1; rcnt <= 7'd0; rstate <= 2'd2;
                end
            end
            2'd2: begin
                rcnt <= rcnt + 1'b1;
                if (rcnt >= DIV - 1) begin                   // 每位中点
                    rshr <= {uart_txd, rshr[7:1]};
                    rcnt <= 7'd0;
                    if (rbit == 3'd7) rstate <= 2'd3;
                    else              rbit  <= rbit + 1'b1;
                end
            end
            2'd3: begin
                rcnt <= rcnt + 1'b1;
                if (rcnt >= DIV - 1) begin                   // 停止位中点
                    rstate <= 2'd0;
                    if (uart_txd) begin
                        rxd_ok = rxd_ok + 1;
                        if (rshr == 8'h0A) begin
                            parse_line(line_chars);
                            line_chars = 0;
                        end else if (line_chars < 64) begin
                            line_buf[line_chars] = rshr;
                            line_chars = line_chars + 1;
                        end
                    end else
                        rxd_badstop = rxd_badstop + 1;
                end
            end
        endcase
    end
end

// ============================================================
// 行解析（含负号）与断言
// ============================================================
integer fld [0:3];
integer nfld;
integer i, v, neg;
reg [7:0] c;

task parse_line;
    input integer n;
    begin
        lines_seen = lines_seen + 1;
        nfld = 0; v = 0; neg = 0;
        for (i = 0; i < n; i = i + 1) begin
            c = line_buf[i];
            if (c >= "0" && c <= "9")
                v = v * 10 + (c - "0");
            else if (c == "-")
                neg = 1;
        end
        fld[0] = neg ? -v : v; nfld = 1;
        check_line;
    end
endtask

task check_line;
    begin
        if (grace > 0) begin
            grace = grace - 1;           // 值切换在途行：跳过
        end else if (nfld !== 1) begin
            $display("[FAIL t=%0t] line %0d fields (expect 1)", $time, nfld);
            errors = errors + 1;
        end else if (adc_data == 12'd1500) begin
            if (fld[0] < -16200 || fld[0] > -16030) begin    // -16113（含舍入带）
                $display("[FAIL t=%0t] v_mV=%0d (expect -16113)", $time, fld[0]);
                errors = errors + 1;
            end
        end else begin
            if (fld[0] < 10680 || fld[0] > 10800) begin      // +10742（含舍入带）
                $display("[FAIL t=%0t] v_mV=%0d (expect 10742)", $time, fld[0]);
                errors = errors + 1;
            end
        end
    end
endtask

// ============================================================
// 主流程
// ============================================================
integer lines_w0;
initial begin
    rst_n = 0;
    #200;
    rst_n = 1;

    // 阶段 1：原始码 1500（行率 ~1000Hz，10ms ≥ 9 行）
    #10_000_000;
    lines_w0 = lines_seen;
    $display("--- Test 1: raw 1500 -> -16113 mV (%0d lines) ---", lines_w0);
    if (lines_w0 < 5) begin
        $display("[FAIL] only %0d lines", lines_w0); errors = errors + 1;
    end

    // 阶段 2：切原始码 2600 → +10742 mV
    grace = 2;
    adc_data = 12'd2600;
    #10_000_000;
    $display("--- Test 2: raw 2600 -> +10742 mV ---");

    $display("[STAT] rxd_ok=%0d badstop=%0d lines=%0d", rxd_ok, rxd_badstop, lines_seen);
    if (errors == 0) $display("=== vofa_tb ALL PASS ===");
    else             $display("=== vofa_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #35_000_000;
    $display("TIMEOUT (errors=%0d)", errors);
    $finish;
end

endmodule
