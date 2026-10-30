// 256 点基-2 DIF FFT（自然顺序输入，位反转输出，读出时反转还原）+ CORDIC 幅度/相位计算
// 数据通路 21bit 有符号（DATA_BITS+9），满幅输入下逐级增长 256 倍不溢出
// 相位输出 16bit：65536 = 360°，幅度输出为 |X[k]|（已做 1/CORDIC 增益校正）
//
// 存储器端口结构（保证 TD 推断 ERAM）：
//   旧版蝶形在 BUTTERFLY 同周期读 addr_a/addr_b 两个地址、STORE_RESULT 同周期写
//   两个地址——每存储器需要 2 读 + 2 写，超出 ERAM9K 端口能力，TD 全部塌成
//   寄存器 + 巨型多路器（约 33.6k LUT，直接超出器件 19600 LUT）。
//   现改为每存储器 1 写端口 + 1 读端口（各一个 always 块，与 display_ctrl
//   的 wf_buf 同构），蝶形串行访问：
//     FFT_STAGE(发 addr_a)→RD_A(发 addr_b)→RD_B(收 a)→RD_TW(收 b)
//     →MUL(差/和)→MUL2(4 个 DSP 乘积)→MUL3(复数合并)→WR_A(写 a)→WR_B(写 b)
//   9 拍/蝶形，1024 蝶形约 9.2k 拍；帧周期 2M 拍（50Hz×256 点@100MHz），占用 <0.5%。
module fft_top #(
    parameter FFT_SIZE = 256,
    parameter DATA_BITS = 12,
    parameter PHASE_BITS = 16
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [DATA_BITS-1:0] data_in,
    input  wire [2:0]          channel_in,
    input  wire                valid_in,
    input  wire                sample_sync,

    output wire [31:0]         mag_out,
    output wire [PHASE_BITS-1:0] phase_out,
    output wire [7:0]          bin_index,
    output wire               fft_done,
    output wire [2:0]         fft_channel,
    output wire               mag_valid,

    input  wire        enable
);

localparam LOG2_FFT_SIZE = 8;
localparam STAGE_NUM = LOG2_FFT_SIZE;
localparam HALF_FFT = FFT_SIZE / 2;
localparam MWD = DATA_BITS + 9;   // 21bit：|X| <= 256*4095 = 1048320 < 2^20

(* ram_style = "block" *) reg signed [MWD-1:0] real_mem [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [MWD-1:0] imag_mem [0:FFT_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] twiddle_rom_real [0:HALF_FFT-1];
(* ram_style = "block" *) reg signed [15:0] twiddle_rom_imag [0:HALF_FFT-1];

initial begin
`include "twiddle_real_init.vh"
`include "twiddle_imag_init.vh"
end

reg [7:0] write_addr;
reg       mem_writing;
reg [2:0] buf_channel;

reg signed [MWD-1:0] real_a, imag_a;
reg signed [MWD-1:0] real_b, imag_b;
reg signed [15:0] twiddle_real, twiddle_imag;

// 蝶形乘法 3 拍流水线（原单拍 21×16 复数乘 + 38bit 合并 17.75ns，超 100MHz 周期
// 5.5ns，全设计 152k 条违例路径）。差值四舍五入到 18bit 后 18×16 完整落进
// MULT18 DSP，消除 TD 用 LUT 分解宽乘法的长链。18bit 舍入误差 ≤4（21bit 标度，
// 仅后期幅度超 2^17 的大值级生效，相对误差 <0.003%），对谐波/相位测量无影响。
(* mult_style = "dsp" *) reg signed [33:0] prod_rr, prod_ii, prod_ri, prod_ir;  // 18×16 → 34bit
reg signed [MWD-1:0]  real_out_a, imag_out_a;
reg signed [MWD-1:0]  real_out_b, imag_out_b;

reg [4:0]  state;
localparam IDLE      = 5'd0;
localparam LOAD_WAIT = 5'd1;
localparam FFT_STAGE = 5'd2;
localparam RD_A      = 5'd3;
localparam RD_B      = 5'd4;
localparam RD_TW     = 5'd5;
localparam MUL       = 5'd6;   // 拍1：差值/和
localparam MUL2      = 5'd7;   // 拍2：4 个 18×16 DSP 乘积
localparam MUL3      = 5'd8;   // 拍3：复数合并 + >>>12 缩放
localparam WR_A      = 5'd9;
localparam WR_B      = 5'd10;
localparam MAG_LOAD  = 5'd11;
localparam MAG_RD    = 5'd12;
localparam MAG_FIX   = 5'd13;
localparam MAG_ITER  = 5'd14;
localparam MAG_EMIT  = 5'd15;
localparam MAG_DONE  = 5'd16;

reg [3:0]  stage_cnt;
reg [7:0]  butterfly_cnt;
reg [7:0]  group_cnt;
reg [7:0]  butterfly_per_group;
reg [8:0]  group_distance;

reg [7:0]  group_base;
reg [7:0]  twiddle_step;

reg [7:0]  addr_a, addr_b;
reg [7:0]  twiddle_index;
reg [7:0]  output_cnt;

// ---------------------------------------------------------------------------
// 存储器端口：real_mem / imag_mem 各 1 写 + 1 读（simple dual-port ERAM）
// 写地址选择：WR_A 写 addr_a、WR_B 写 addr_b、IDLE 装载写 write_addr
// ---------------------------------------------------------------------------
wire        mem_wen = (state == WR_A) || (state == WR_B) ||
                     ((state == IDLE) && enable && mem_writing && valid_in);
wire [7:0]  mem_waddr = (state == WR_A) ? addr_a :
                        (state == WR_B) ? addr_b : write_addr;
wire signed [MWD-1:0] real_wdata = (state == WR_A) ? real_out_a :
                                   (state == WR_B) ? real_out_b :
                                   {{(MWD-DATA_BITS){1'b0}}, data_in};
wire signed [MWD-1:0] imag_wdata = (state == WR_A) ? imag_out_a :
                                   (state == WR_B) ? imag_out_b : {MWD{1'b0}};

always @(posedge clk) begin
    if (mem_wen) begin
        real_mem[mem_waddr] <= real_wdata;
        imag_mem[mem_waddr] <= imag_wdata;
    end
end

// 读端口：读数据 1 拍后有效；无条件同步读（与 display_ctrl 的 wf_buf 完全同构，
// 确保 TD 推断 ERAM）。写周期读到的垃圾数据不会被任何状态采样，功能不受影响。
reg [7:0] mem_raddr;
reg signed [MWD-1:0] rmem_q, imem_q;

always @(posedge clk) begin
    rmem_q <= real_mem[mem_raddr];
    imem_q <= imag_mem[mem_raddr];
end

// 旋转因子读端口（ROM 只读，1 拍延迟；twiddle_index 在整个蝶形期间稳定）
always @(posedge clk) begin
    twiddle_real <= twiddle_rom_real[twiddle_index];
    twiddle_imag <= twiddle_rom_imag[twiddle_index];
end

reg fft_done_reg;
reg [2:0] fft_channel_reg;
reg mag_valid_reg;
reg [31:0] mag_out_reg;
reg [PHASE_BITS-1:0] phase_out_reg;
reg [7:0] bin_index_reg;

reg signed [MWD-1:0] real_diff, imag_diff;

// 拍2 输入：差值四舍五入到 18bit（×8 补偿在拍3 用 >>>12 实现）
wire signed [20:0] real_diff_r = (real_diff + 21'sd4) >>> 3;
wire signed [20:0] imag_diff_r = (imag_diff + 21'sd4) >>> 3;
wire signed [17:0] rd18 = real_diff_r[17:0];
wire signed [17:0] id18 = imag_diff_r[17:0];

reg sample_sync_d1;
wire sample_start = sample_sync && !sample_sync_d1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sample_sync_d1 <= 1'b0;
    else sample_sync_d1 <= sample_sync;
end

// 帧起点对齐：模 256 采样脉冲计数，只在计数归零的脉冲开始捕获。
// FFT 处理耗时（约 100µs）大于一个采样周期（78µs），若处理完就立即开下一帧，
// 帧间隔会是 257/258 个脉冲——不是信号周期的整数倍，B/C 相相位相对 A 相多出
// 1.4°/脉冲的系统偏差（上板实测 PH 显示 118~123 跳动）。256 采样 = 1 信号周期，
// 对齐后帧间隔恒为 256 的整数倍，相位差精确回到 ±120°。
reg [7:0] samp_phase_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) samp_phase_cnt <= 8'd0;
    else if (sample_start) samp_phase_cnt <= samp_phase_cnt + 1'b1;
end

// CORDIC 向量模式：迭代求 atan2 与幅度
reg signed [MWD+1:0] cord_x, cord_y;    // 23bit：K*sqrt(2)*2^20 有余量
reg signed [17:0]   cord_z;             // 角度累加（65536 = 360°）
reg signed [MWD-1:0] m_re, m_im;
reg [3:0] cord_i;

// atan(2^-i) 换算到 65536=2π 单位
function [15:0] atan_tab;
    input [3:0] i;
    case (i)
        4'd0:  atan_tab = 16'd8192;    // atan(1)=45°，此前误写 16384(90°) 导致相位恒偏 +45°
        4'd1:  atan_tab = 16'd4835;
        4'd2:  atan_tab = 16'd2555;
        4'd3:  atan_tab = 16'd1297;
        4'd4:  atan_tab = 16'd651;
        4'd5:  atan_tab = 16'd326;
        4'd6:  atan_tab = 16'd163;
        4'd7:  atan_tab = 16'd81;
        4'd8:  atan_tab = 16'd41;
        4'd9:  atan_tab = 16'd20;
        4'd10: atan_tab = 16'd10;
        4'd11: atan_tab = 16'd5;
        4'd12: atan_tab = 16'd3;
        4'd13: atan_tab = 16'd1;
        4'd14: atan_tab = 16'd1;
        default: atan_tab = 16'd0;
    endcase
endfunction

// DIF 输出位反转：位置 p 存放 X[bitrev(p)]，按自然 bin 序读出
function [7:0] bitrev8;
    input [7:0] d;
    begin
        bitrev8 = {d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]};
    end
endfunction

// 幅度 = x_final / K，K=1.64676，1/K=0.60725 → ×39797>>16
wire signed [40:0] mag_prod = cord_x * 17'sd39797;

// CORDIC 迭代（向量模式，z 累加为正角度约定）
wire signed [MWD+1:0] cx_sh = cord_y >>> cord_i;
wire signed [MWD+1:0] cy_sh = cord_x >>> cord_i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= IDLE;
        write_addr      <= 8'd0;
        mem_writing     <= 1'b0;
        buf_channel     <= 3'd0;
        stage_cnt       <= 4'd0;
        butterfly_cnt   <= 8'd0;
        group_cnt       <= 8'd0;
        fft_done_reg    <= 1'b0;
        fft_channel_reg <= 3'd0;
        mag_valid_reg   <= 1'b0;
        mag_out_reg     <= 32'd0;
        phase_out_reg   <= 16'd0;
        bin_index_reg   <= 8'd0;
        output_cnt      <= 8'd0;
        real_diff       <= {MWD{1'b0}};
        imag_diff       <= {MWD{1'b0}};
        mem_raddr       <= 8'd0;
    end else begin
        mag_valid_reg <= 1'b0;

        case (state)
            IDLE: begin
                fft_done_reg <= 1'b0;
                if (enable) begin
                    if (!mem_writing) begin
                        if (sample_start && (samp_phase_cnt == 8'd0)) begin
                            mem_writing <= 1'b1;
                            write_addr  <= 8'd0;
                        end
                    end else begin
                        if (valid_in) begin
                            buf_channel <= channel_in;
                            if (write_addr >= FFT_SIZE - 1) begin
                                state <= LOAD_WAIT;
                                mem_writing <= 1'b0;
                            end else begin
                                write_addr <= write_addr + 1'b1;
                            end
                        end
                    end
                end
            end

            LOAD_WAIT: begin
                fft_channel_reg     <= buf_channel;   // 发送幅度前先更新通道，避免 measurement 归属错位
                state               <= FFT_STAGE;
                stage_cnt           <= 4'd0;
                butterfly_cnt       <= 8'd0;
                group_cnt           <= 8'd0;
                butterfly_per_group <= HALF_FFT[7:0];  // DIF：第1级蝶距128，逐级减半
                group_distance      <= HALF_FFT[8:0];
                group_base          <= 8'd0;
                twiddle_step        <= 8'd1;            // DIF：第1级步进1，逐级倍增
                twiddle_index       <= 8'd0;
            end

            FFT_STAGE: begin
                if (stage_cnt < STAGE_NUM) begin
                    addr_a    <= group_base + butterfly_cnt;
                    addr_b    <= group_base + butterfly_cnt + butterfly_per_group;
                    mem_raddr <= group_base + butterfly_cnt;   // 先发 a 地址
                    if (butterfly_cnt == 8'd0)
                        twiddle_index <= 8'd0;
                    state <= RD_A;
                end else begin
                    output_cnt <= 8'd0;
                    mem_raddr  <= bitrev8(8'd0);
                    state      <= MAG_LOAD;
                end
            end

            RD_A: begin
                mem_raddr <= addr_b;    // a 数据在读端口加载中，本拍发 b 地址
                state <= RD_B;
            end

            RD_B: begin
                real_a <= rmem_q;       // = mem[addr_a]
                imag_a <= imem_q;
                state <= RD_TW;
            end

            RD_TW: begin
                real_b <= rmem_q;       // = mem[addr_b]
                imag_b <= imem_q;
                state <= MUL;
            end

            MUL: begin
                real_diff <= real_a - real_b;
                imag_diff <= imag_a - imag_b;
                real_out_a <= real_a + real_b;
                imag_out_a <= imag_a + imag_b;
                state <= MUL2;
            end

            MUL2: begin
                prod_rr <= rd18 * twiddle_real;
                prod_ii <= id18 * twiddle_imag;
                prod_ri <= rd18 * twiddle_imag;
                prod_ir <= id18 * twiddle_real;
                state <= MUL3;
            end

            MUL3: begin
                // rd18 = real_diff/8（四舍五入），×8 补偿后等效原 >>>15 → >>>12
                real_out_b <= (prod_rr - prod_ii + 35'sd2048) >>> 12;
                imag_out_b <= (prod_ri + prod_ir + 35'sd2048) >>> 12;
                state <= WR_A;
            end

            WR_A: begin
                state <= WR_B;         // 写端口本拍写 addr_a
            end

            WR_B: begin
                // 写端口本拍写 addr_b
                if (butterfly_cnt >= butterfly_per_group - 1) begin
                    butterfly_cnt <= 8'd0;
                    // FFT_SIZE[7:0] 会截断为 0（256 超出 8bit），必须用 9bit 比较，
                    // 否则每级只算第 1 组蝶形就跳级
                    if (group_base + (butterfly_per_group << 1) >= FFT_SIZE[8:0]) begin
                        group_cnt   <= 8'd0;
                        group_base  <= 8'd0;
                        stage_cnt   <= stage_cnt + 1'b1;
                        butterfly_per_group <= butterfly_per_group >> 1;
                        group_distance      <= group_distance << 1;
                        twiddle_step        <= twiddle_step << 1;
                    end else begin
                        group_cnt  <= group_cnt + 1'b1;
                        group_base <= group_base + (butterfly_per_group << 1);
                    end
                end else begin
                    butterfly_cnt <= butterfly_cnt + 1'b1;
                    twiddle_index <= twiddle_index + twiddle_step;
                end
                state <= FFT_STAGE;
            end

            MAG_LOAD: begin
                state <= MAG_RD;       // 等读数据有效
            end

            MAG_RD: begin
                m_re  <= rmem_q;       // = mem[bitrev8(output_cnt)]
                m_im  <= imem_q;
                state <= MAG_FIX;
            end

            MAG_FIX: begin
                cord_i <= 4'd0;
                if (m_re < 0) begin
                    // 象限预旋转 180°：两分量取反，初始角 = 32768 (180°)
                    cord_x <= -m_re;
                    cord_y <= -m_im;
                    cord_z <= 18'sd32768;
                end else begin
                    cord_x <= m_re;
                    cord_y <= m_im;
                    cord_z <= 18'sd0;
                end
                state <= MAG_ITER;
            end

            MAG_ITER: begin
                if (cord_y >= 0) begin
                    cord_x <= cord_x + cx_sh;
                    cord_y <= cord_y - cy_sh;
                    cord_z <= cord_z + {2'b0, atan_tab(cord_i)};
                end else begin
                    cord_x <= cord_x - cx_sh;
                    cord_y <= cord_y + cy_sh;
                    cord_z <= cord_z - {2'b0, atan_tab(cord_i)};
                end
                if (cord_i == 4'd15)
                    state <= MAG_EMIT;
                else
                    cord_i <= cord_i + 1'b1;
            end

            MAG_EMIT: begin
                mag_out_reg   <= mag_prod >>> 16;
                phase_out_reg <= cord_z[15:0];
                bin_index_reg <= output_cnt;
                mag_valid_reg <= 1'b1;
                if (output_cnt >= HALF_FFT - 1) begin
                    state <= MAG_DONE;
                end else begin
                    output_cnt <= output_cnt + 1'b1;
                    mem_raddr  <= bitrev8(output_cnt + 8'd1);
                    state      <= MAG_LOAD;
                end
            end

            MAG_DONE: begin
                fft_done_reg    <= 1'b1;
                fft_channel_reg <= buf_channel;
                mem_writing     <= 1'b0;
                write_addr      <= 8'd0;
                state           <= IDLE;
            end

            default: state <= IDLE;
        endcase
    end
end

assign mag_out     = mag_out_reg;
assign phase_out   = phase_out_reg;
assign bin_index   = bin_index_reg;
assign fft_done    = fft_done_reg;
assign fft_channel = fft_channel_reg;
assign mag_valid   = mag_valid_reg;

endmodule
