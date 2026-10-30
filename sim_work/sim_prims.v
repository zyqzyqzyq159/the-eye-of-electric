`timescale 1ns / 1ps
// Anlogic 原语行为模型（仅用于 ModelSim 仿真，不参与综合）
//  - EG_LOGIC_BUFG ：时钟缓冲
//  - EG_LOGIC_ODDR ：DDR 输出寄存器
//  - EG_PHY_PLL    ：PLL（按 REFCLK_DIV/FBCLK_DIV/CLKCx_DIV 推算输出频率）
//    约定：C0 = FIN*FBCLK_DIV/REFCLK_DIV，VCO = C0*CLKC0_DIV，Ci = VCO/CLKCi_DIV

module EG_LOGIC_BUFG (i, o);
    input  i;
    output o;
    assign o = i;
endmodule

module EG_LOGIC_ODDR #(
    parameter ASYNCRST = "ENABLE"
) (
    input  clk,
    input  d1,
    input  d0,
    input  rst,
    output reg q
);

    always @(clk or posedge rst) begin
        if (rst)     q = 1'b0;
        else if (clk) q = d1;
        else          q = d0;
    end
endmodule

module EG_PHY_PLL #(
    parameter DPHASE_SOURCE = "DISABLE",
    parameter DYNCFG        = "DISABLE",
    parameter FIN           = "25.000000",
    parameter FEEDBK_MODE   = "NORMAL",
    parameter FEEDBK_PATH   = "CLKC0_EXT",
    parameter STDBY_ENABLE = "DISABLE",
    parameter PLLRST_ENA    = "ENABLE",
    parameter SYNC_ENABLE   = "DISABLE",
    parameter GMC_GAIN      = 2,
    parameter ICP_CURRENT   = 9,
    parameter KVCO          = 2,
    parameter LPF_CAPACITOR = 1,
    parameter LPF_RESISTOR  = 8,
    parameter REFCLK_DIV    = 1,
    parameter FBCLK_DIV     = 1,
    parameter CLKC0_ENABLE  = "ENABLE",
    parameter CLKC0_DIV     = 2,
    parameter CLKC0_CPHASE  = 1,
    parameter CLKC0_FPHASE  = 0,
    parameter CLKC1_ENABLE  = "DISABLE",
    parameter CLKC1_DIV     = 2,
    parameter CLKC1_CPHASE  = 1,
    parameter CLKC1_FPHASE  = 0,
    parameter CLKC2_ENABLE  = "DISABLE",
    parameter CLKC2_DIV     = 2,
    parameter CLKC2_CPHASE  = 1,
    parameter CLKC2_FPHASE  = 0
) (
    input  wire       refclk,
    input  wire       reset,
    input  wire       stdby,
    output reg        extlock,
    input  wire       load_reg,
    input  wire       psclk,
    input  wire       psdown,
    input  wire       psstep,
    input  wire [2:0] psclksel,
    output wire       psdone,
    input  wire       dclk,
    input  wire       dcs,
    input  wire       dwe,
    input  wire [7:0] di,
    input  wire [5:0] daddr,
    output wire [7:0] do,
    input  wire       fbclk,
    output wire [4:0] clkc
);

assign psdone = 1'b0;
assign do     = 8'h00;

// 上电时 extlock=0（模拟真实 PLL 未锁定状态，避免下游 X 传播）
initial extlock = 1'b0;

real t_prev, ref_period;
real c0_half, c1_half, c2_half;

initial begin
    t_prev     = 0.0;
    ref_period = 40.0;
    c0_half    = 20.0;
    c1_half    = 6.67;
    c2_half    = 1.33;
end

always @(posedge refclk) begin
    if (t_prev > 0.0) begin
        ref_period = $realtime - t_prev;
        c0_half = ref_period * REFCLK_DIV / FBCLK_DIV / 2.0;
        c1_half = c0_half * CLKC1_DIV / CLKC0_DIV;
        c2_half = c0_half * CLKC2_DIV / CLKC0_DIV;
    end
    t_prev = $realtime;
end

reg clkc0_r = 1'b0;
reg clkc1_r = 1'b0;
reg clkc2_r = 1'b0;

assign clkc = {2'b00, clkc2_r, clkc1_r, clkc0_r};

initial begin
    @(posedge refclk); @(posedge refclk);
    forever #(c0_half) clkc0_r = ~clkc0_r;
end

initial begin
    @(posedge refclk); @(posedge refclk);
    forever #(c1_half) clkc1_r = ~clkc1_r;
end

initial begin
    @(posedge refclk); @(posedge refclk);
    forever #(c2_half) clkc2_r = ~clkc2_r;
end

reg [5:0] lockcnt = 6'd0;
always @(posedge refclk or posedge reset) begin
    if (reset) begin
        lockcnt <= 6'd0;
        extlock <= 1'b0;
    end else if (lockcnt < 6'd50) begin
        lockcnt <= lockcnt + 1'b1;
    end else begin
        extlock <= 1'b1;
    end
end

endmodule
