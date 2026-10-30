// VOFA+ 遥测流（V35：A 相 SPI 原始帧电压遥测，纯 TX 单模式）
//
// 每行：±mmmmm\n —— A 相电压毫伏值（带符号，±20V 输入显示 ±20000）
//   数据源 = ADC 原始帧流（中值滤波**之前**——SPI 采到什么发什么，坏帧/串台
//   原样可见）；×24.4140625 mV/码 = ×3125>>7（2048+1024+32+16+4+1 移位加，
//   零乘法器，DSP 不占）。
//   行率 = 100MHz 独立分频定时器恒 LINE_RATE_HZ（默认 1000 行/秒：VOFA Δt=1
//   即真实时间，长期精确无漂移）。
//
// V35 变更（突发同步采样配套）：V33 按帧率分数抽选（A 帧率 19.2kHz 已知恒定）；
// top.v 改栅格触发突发采样后 A 相帧率被 PLL 栅格锁定 = 256×工频（45~55Hz 时
// 11.5~14.1kHz），按帧率抽选的行率会随工频漂移 ±10%。改为与 V31.5 同源的
// 独立定时器机制（该机制已在实机验证过 Δt=1 时间轴真实）：每 LINE_RATE_HZ
// 发一行"最新 A 相帧"快照，行率与信号频率完全解耦。
//
// V28 基线零改动：本模块纯旁路（只读 adc_* 总线 + 一根 TX 线），测量/显示/
// 锁相路径一字节不碰。波特率 921600（uart_tx 参数同步）。
module vofa_stream #(
    parameter CLK_FREQ       = 100_000_000,
    parameter BAUD           = 921600,
    parameter LINE_RATE_HZ   = 1000,     // UART 行率（VOFA Δt=1 即真实时间）
    parameter DC_BIAS_CODE   = 2160      // 采样板偏置码（与 top.v 直流常数同源）
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [11:0] adc_data,         // ADC 原始帧（中值滤波前）
    input  wire [2:0]  adc_channel,
    input  wire        adc_valid,

    output wire        uart_txd
);

    // --- A 相帧选通 ---
    wire a_frame = adc_valid && (adc_channel == 3'd0);

    // --- 原始码 差/符号（无符号减法，平凡写法） ---
    wire        va_lt  = (adc_data < DC_BIAS_CODE[11:0]);
    wire [11:0] va_abs = va_lt ? (DC_BIAS_CODE[11:0] - adc_data)
                               : (adc_data - DC_BIAS_CODE[11:0]);

    // --- 独立行率定时器：每 LINE_RATE_HZ 锁一次"最新 A 相帧"快照并发一行 ---
    // hold_* 随每个 A 帧刷新；定时到拍把快照转存 s_* 并启动发送 FSM——
    // 行发送期间（~76µs）新到帧只更新 hold_*，不窜行内数值
    localparam [16:0] EMIT_DIV = CLK_FREQ / LINE_RATE_HZ - 1;   // 100MHz/1000-1=99_999
    reg [16:0] emit_cnt;
    reg        hold_sign;
    reg [11:0] hold_abs;
    reg        s_sign;
    reg [11:0] s_abs;
    reg        wf_trig;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            emit_cnt  <= 17'd0;
            hold_sign <= 1'b0;
            hold_abs  <= 12'd0;
            s_sign    <= 1'b0;
            s_abs     <= 12'd0;
            wf_trig   <= 1'b0;
        end else begin
            wf_trig <= 1'b0;
            if (a_frame) begin
                hold_sign <= va_lt;
                hold_abs  <= va_abs;
            end
            if (emit_cnt >= EMIT_DIV) begin
                emit_cnt <= 17'd0;
                s_sign   <= hold_sign;
                s_abs    <= hold_abs;
                wf_trig  <= 1'b1;
            end else begin
                emit_cnt <= emit_cnt + 1'b1;
            end
        end
    end

    // --- 毫伏换算：×3125>>7 = ×24.4140625（s_abs ≤2160 → 6,750,000 < 2^23） ---
    wire [22:0] mv_tree = ({11'd0, s_abs} << 11) + ({11'd0, s_abs} << 10)
                        + ({11'd0, s_abs} << 5)  + ({11'd0, s_abs} << 4)
                        + ({11'd0, s_abs} << 2)  + {11'd0, s_abs};
    wire [15:0] mv_val  = mv_tree[22:7];          // ≤52734

    // --- 发送 FSM：7 槽 "+/-mmmmm\n"（c0 符号 c1..c5 数字 c6 换行） ---
    localparam S_IDLE = 3'd0, S_CHAR = 3'd1, S_MUL = 3'd2,
               S_DIV  = 3'd3, S_SEND = 3'd4, S_NEXT = 3'd5;
    reg [2:0]  fsm;
    reg [5:0]  c_idx;
    reg [15:0] dv;
    reg [2:0]  pos_r;
    reg [7:0]  char_r;
    wire tx_busy;

    wire [15:0] dv_div10 = dv / 16'd10;  // 常数无符号除法（综合为移位加，1 份）
    wire [15:0] dv_mod10 = dv % 16'd10;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm    <= S_IDLE;
            c_idx  <= 6'd0;
            dv     <= 16'd0;
            pos_r  <= 3'd0;
            char_r <= 8'd0;
        end else begin
            case (fsm)
                S_IDLE: begin
                    if (wf_trig) begin
                        c_idx <= 6'd0;
                        fsm   <= S_CHAR;
                    end
                end

                S_CHAR: begin
                    fsm <= S_MUL;
                    case (c_idx)
                        6'd0: begin
                            char_r <= s_sign ? 8'h2D : 8'h2B;   // '-' / '+'
                            fsm    <= S_SEND;
                        end
                        6'd1: begin pos_r <= 3'd4; end
                        6'd2: begin pos_r <= 3'd3; end
                        6'd3: begin pos_r <= 3'd2; end
                        6'd4: begin pos_r <= 3'd1; end
                        6'd5: begin pos_r <= 3'd0; end
                        default: begin
                            char_r <= 8'h0A;                    // c6 行尾
                            fsm    <= S_SEND;
                        end
                    endcase
                end

                // 毫伏移位加树独立一拍（23bit 加法链不与 FSM 选择叠加）
                S_MUL: begin
                    dv  <= mv_val;
                    fsm <= S_DIV;
                end

                S_DIV: begin
                    if (pos_r == 3'd0) begin
                        char_r <= dv_mod10[7:0] + 8'h30;
                        fsm    <= S_SEND;
                    end else begin
                        dv    <= dv_div10;
                        pos_r <= pos_r - 3'd1;
                    end
                end

                S_SEND: begin
                    if (!tx_busy)
                        fsm <= S_NEXT;      // char_wr 由独立时序块置位
                end

                S_NEXT: begin
                    if (c_idx == 6'd6)
                        fsm <= S_IDLE;
                    else begin
                        c_idx <= c_idx + 6'd1;
                        fsm   <= S_CHAR;
                    end
                end

                default: fsm <= S_IDLE;
            endcase
        end
    end

    // char_wr：SEND 拍（!tx_busy）置位、NEXT 拍撤销（uart_tx 在 !busy 采样一拍）
    reg char_wr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                            char_wr <= 1'b0;
        else if (fsm == S_SEND && !tx_busy)    char_wr <= 1'b1;
        else if (fsm == S_NEXT)                char_wr <= 1'b0;
    end

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) tx_inst (
        .clk(clk), .rst_n(rst_n),
        .din(char_r), .wr(char_wr), .tx(uart_txd), .busy(tx_busy)
    );

endmodule
