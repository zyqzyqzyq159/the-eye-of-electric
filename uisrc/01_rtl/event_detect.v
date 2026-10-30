// 暂降/暂升/中断检测（V26 资源减法版）
// 原 64 条事件环形缓存（evt_type/evt_phase/evt_ts/evt_dur 四块 BRAM + 9 处变址
// 写 + 读指针）的读出端口在 top 级全部悬空——无任何 UI/逻辑消费，属不可观察
// 的死存储，已删除（省 4 BRAM 与数百 LUT）。检测/消抖确认/报警/事件计数的
// 可观察行为与删除前逐位一致；需要事件列表 UI 时从 git 历史取回缓存实现。
module event_detect #(
    parameter RMS_BITS = 32
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [RMS_BITS-1:0] rms_va,
    input  wire [RMS_BITS-1:0] rms_vb,
    input  wire [RMS_BITS-1:0] rms_vc,
    input  wire               measure_done,

    input  wire [31:0] nominal_voltage,
    input  wire [7:0]  sag_threshold,
    input  wire [7:0]  swell_threshold,
    input  wire [7:0]  interrupt_threshold,

    output wire [2:0]  event_alarm,
    output wire [31:0] event_count,

    input  wire        enable
);

wire [31:0] sag_thr   = (nominal_voltage * sag_threshold) >> 7;
wire [31:0] swell_thr = (nominal_voltage * swell_threshold) >> 7;
wire [31:0] int_thr   = (nominal_voltage * interrupt_threshold) >> 7;

reg [RMS_BITS-1:0] rms_arr [0:2];

reg [2:0]  sag_det, swell_det, int_det;
reg [2:0]  sag_conf, swell_conf, int_conf;
reg [15:0] sag_cnt   [0:2];
reg [15:0] swell_cnt [0:2];
reg [15:0] int_cnt   [0:2];

reg [31:0] event_count_reg;

reg [7:0]  warmup_cnt;
wire       warmup_done = (warmup_cnt >= 8'd50);

integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 3; i = i + 1) begin
            sag_det[i] <= 1'b0; swell_det[i] <= 1'b0; int_det[i] <= 1'b0;
            sag_conf[i] <= 1'b0; swell_conf[i] <= 1'b0; int_conf[i] <= 1'b0;
            sag_cnt[i] <= 16'd0; swell_cnt[i] <= 16'd0; int_cnt[i] <= 16'd0;
            rms_arr[i] <= 32'd0;
        end
        event_count_reg <= 32'd0;
        warmup_cnt <= 8'd0;
    end else begin
        if (measure_done && enable && !warmup_done)
            warmup_cnt <= warmup_cnt + 1'b1;
        rms_arr[0] <= rms_va;
        rms_arr[1] <= rms_vb;
        rms_arr[2] <= rms_vc;

        if (measure_done && enable && warmup_done) begin
            for (i = 0; i < 3; i = i + 1) begin
                if (rms_arr[i] < sag_thr && rms_arr[i] > int_thr) begin
                    if (!sag_det[i]) begin
                        sag_det[i] <= 1'b1; sag_cnt[i] <= 16'd1;
                    end else begin
                        sag_cnt[i] <= sag_cnt[i] + 1'b1;
                        if (sag_cnt[i] >= 16'd10 && !sag_conf[i]) begin
                            sag_conf[i] <= 1'b1;
                            event_count_reg <= event_count_reg + 1'b1;
                        end
                    end
                end else begin
                    sag_det[i] <= 1'b0; sag_cnt[i] <= 16'd0; sag_conf[i] <= 1'b0;
                end

                if (rms_arr[i] > swell_thr) begin
                    if (!swell_det[i]) begin
                        swell_det[i] <= 1'b1; swell_cnt[i] <= 16'd1;
                    end else begin
                        swell_cnt[i] <= swell_cnt[i] + 1'b1;
                        if (swell_cnt[i] >= 16'd10 && !swell_conf[i]) begin
                            swell_conf[i] <= 1'b1;
                            event_count_reg <= event_count_reg + 1'b1;
                        end
                    end
                end else begin
                    swell_det[i] <= 1'b0; swell_cnt[i] <= 16'd0; swell_conf[i] <= 1'b0;
                end

                if (rms_arr[i] < int_thr) begin
                    if (!int_det[i]) begin
                        int_det[i] <= 1'b1; int_cnt[i] <= 16'd1;
                    end else begin
                        int_cnt[i] <= int_cnt[i] + 1'b1;
                        if (int_cnt[i] >= 16'd5 && !int_conf[i]) begin
                            int_conf[i] <= 1'b1;
                            event_count_reg <= event_count_reg + 1'b1;
                        end
                    end
                end else begin
                    int_det[i] <= 1'b0; int_cnt[i] <= 16'd0; int_conf[i] <= 1'b0;
                end
            end
        end
    end
end

assign event_alarm = {int_conf[0]|int_conf[1]|int_conf[2],
                      swell_conf[0]|swell_conf[1]|swell_conf[2],
                      sag_conf[0]|sag_conf[1]|sag_conf[2]};
assign event_count   = event_count_reg;

endmodule
