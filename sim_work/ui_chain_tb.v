`timescale 1ns / 1ps
// 真实链路 UI 渲染 TB：dds_signal(实际相位累进) → 7813 拍采样分频 → display_ctrl
// 完整复刻 top.v 自检模式的波形馈送（toggle 握手 + 数据先稳 2 拍），
// 验证硬件等效条件下 Ia 等迹线与自适应刻度是否正确显示。
module ui_chain_tb;

reg clk = 1'b0;
always #5 clk = ~clk;                        // 100MHz 采样域
reg clk_px = 1'b0;
always #6.734 clk_px = ~clk_px;              // 74.25MHz 像素域

reg rst_n = 1'b0;

// ---- DDS 自检源（与 top.v 同参数）----
wire [11:0] dds_va, dds_vb, dds_vc, dds_ia, dds_ib, dds_ic;
wire dds_valid;
reg [15:0] amp_cfg = 16'd2048;

dds_signal #(
    .CLK_FREQ(100_000_000), .DAC_BITS(12), .LUT_SIZE(256)
) dds_i (
    .clk(clk), .rst_n(rst_n),
    .dds_va(dds_va), .dds_vb(dds_vb), .dds_vc(dds_vc),
    .dds_ia(dds_ia), .dds_ib(dds_ib), .dds_ic(dds_ic), .dds_valid(dds_valid),
    .freq_config(32'd50), .amp_config(amp_cfg),
    .enable(1'b1), .select(1'b1),
    .distort_en(1'b0), .unbal_en(1'b0), .rev_seq(1'b0)
);

// ---- 7813 拍采样分频 + 波形馈送（复刻 top.v）----
reg [15:0] samp_cnt;
wire samp_pulse = (samp_cnt >= 16'd7812);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) samp_cnt <= 16'd0;
    else if (samp_pulse) samp_cnt <= 16'd0;
    else samp_cnt <= samp_cnt + 1'b1;
end

reg        wf_toggle;
reg [1:0]  wf_tg_dly;
reg [11:0] wf_va_r, wf_vb_r, wf_vc_r, wf_ia_r, wf_ib_r, wf_ic_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wf_toggle <= 1'b0; wf_tg_dly <= 2'd0;
        wf_va_r <= 0; wf_vb_r <= 0; wf_vc_r <= 0;
        wf_ia_r <= 0; wf_ib_r <= 0; wf_ic_r <= 0;
    end else begin
        wf_tg_dly <= {wf_tg_dly[0], samp_pulse};
        if (samp_pulse) begin
            wf_va_r <= dds_va; wf_vb_r <= dds_vb; wf_vc_r <= dds_vc;
            wf_ia_r <= dds_ia; wf_ib_r <= dds_ib; wf_ic_r <= dds_ic;
        end
        if (wf_tg_dly[1]) wf_toggle <= ~wf_toggle;
    end
end

// ---- 测量量喂入（自检满幅标定值，真实硬件测量约 0.24s 后即为此值）----
reg [31:0] rms_va = 32'd351, rms_vb = 32'd351, rms_vc = 32'd351;
reg [31:0] rms_ia = 32'd133, rms_ib = 32'd133, rms_ic = 32'd133;
reg [31:0] thd_va = 32'd0, thd_vb = 32'd0, thd_vc = 32'd0;
reg [31:0] thd_ia = 32'd0, thd_ib = 32'd0, thd_ic = 32'd0;
reg [31:0] freq = 32'd499;
reg [239:0] harm = 240'd0;
reg [15:0] pab = -16'sd21845, pbc = -16'sd21845, pca = -16'sd21845;
reg [15:0] pia = -16'sd5461, pib = -16'sd5461, pic = -16'sd5461;
reg seq = 1'b1;
reg [31:0] unb = 32'd0;
reg done_t = 1'b0;

// ---- 显示控制器（像素域复位与 top.v 一致）----
reg rst_n_meta, rst_n_px;
always @(posedge clk_px or negedge rst_n) begin
    if (!rst_n) begin rst_n_meta <= 0; rst_n_px <= 0; end
    else begin rst_n_meta <= 1'b1; rst_n_px <= rst_n_meta; end
end

wire vid_hs, vid_vs, vid_de;
wire [7:0] rgb_r, rgb_g, rgb_b;
wire [177:0] pwr_bus = 178'd0;

display_ctrl #(
    .H_RES(1280), .V_RES(720), .DATA_BITS(12), .HARMONICS_NUM(15)
) dut (
    .clk(clk_px), .rst_n(rst_n_px),
    .vid_hs(vid_hs), .vid_vs(vid_vs), .vid_de(vid_de),
    .rgb_r(rgb_r), .rgb_g(rgb_g), .rgb_b(rgb_b),
    .rms_va(rms_va), .rms_vb(rms_vb), .rms_vc(rms_vc),
    .rms_ia(rms_ia), .rms_ib(rms_ib), .rms_ic(rms_ic),
    .thd_va(thd_va), .thd_vb(thd_vb), .thd_vc(thd_vc),
    .thd_ia(thd_ia), .thd_ib(thd_ib), .thd_ic(thd_ic),
    .fundamental_freq(freq),
    .harm_pct_va(harm), .harm_pct_vb(harm), .harm_pct_vc(harm),
    .harm_pct_ia(harm), .harm_pct_ib(harm), .harm_pct_ic(harm),
    .phase_ab(pab), .phase_bc(pbc), .phase_ca(pca),
    .phase_ia(pia), .phase_ib(pib), .phase_ic(pic),
    .phase_sequence(seq), .unbalance_degree(unb),
    .done_toggle(done_t),
    .wf_toggle(wf_toggle),
    .wf_va(wf_va_r), .wf_vb(wf_vb_r), .wf_vc(wf_vc_r),
    .wf_ia(wf_ia_r), .wf_ib(wf_ib_r), .wf_ic(wf_ic_r),
    .mode_toggle(1'b0),
    .freeze(1'b0),
    .event_alarm(3'd0),
    .func_toggle(1'b0),
    .func_data(11'd0),
    .view_toggle(1'b0),
    .view_data(45'd0),
    .pwr_bus(pwr_bus),
    .enable(1'b1)
);

// ---- 帧捕获 ----
reg cap = 1'b0;
reg [10:0] px = 0, py = 0;
reg [23:0] frames = 0;
reg [23:0] fb [0:921599];
always @(posedge clk_px) begin
    if (vid_de) begin
        if (cap) fb[py * 1280 + px] <= {rgb_r, rgb_g, rgb_b};
        if (px == 11'd1279) begin
            px <= 0;
            if (py == 11'd719) begin py <= 0; frames <= frames + 1; end
            else py <= py + 1;
        end else px <= px + 1;
    end
end

task dump_ppm(input [8*24-1:0] name);
    integer f, i;
    begin
        f = $fopen(name, "wb");
        $fwrite(f, "P6\n1280 720\n255\n");
        for (i = 0; i < 921600; i = i + 1)
            $fwrite(f, "%c%c%c", fb[i][23:16], fb[i][15:8], fb[i][7:0]);
        $fclose(f);
        $display("dumped %0s", name);
    end
endtask

initial begin
    rst_n = 0;
    #200;
    rst_n = 1;
    // 等 20ms：off 环填满 + 至少一个完整帧
    #20_000_000;  // 20ms：填满样本环（4ms）+ 完整帧后再抓帧
    // 抓一帧
    wait (frames[0] == 1'b0);
    @(negedge clk_px);
    wait (frames[0] == 1'b1);
    cap = 1'b1;
    wait (frames[0] == 1'b0);
    cap = 1'b0;
    dump_ppm("ui_chain.ppm");
    // 顺带探针：off_ram 前 4 字 + Ia 通道偏移
    $display("off_ram0=%h off_ram1=%h off_ram2=%h off_ram3=%h",
             dut.off_ram[0], dut.off_ram[1], dut.off_ram[2], dut.off_ram[3]);
    $display("sclV=%0d/%0d/%0d sclI=%0d/%0d/%0d",
             dut.scl_v_r[0], dut.scl_v_r[1], dut.scl_v_r[2],
             dut.scl_i_r[0], dut.scl_i_r[1], dut.scl_i_r[2]);
    $finish;
end

initial begin
    #80_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
