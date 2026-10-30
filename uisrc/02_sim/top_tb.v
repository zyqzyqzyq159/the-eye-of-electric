`timescale 1ns / 1ps

module top_tb;

reg         SYSCLK_25M;
reg         ADC_MISO;
reg         BTN_SW2;
reg         BTN_SW3;
reg         BTN_FUNC;
reg         BTN_VIEW;
reg         BTN_FREEZE;
reg         BTN_SEL1;
reg         BTN_SEL2;
reg         BTN_SEL3;
reg         ENC1_CHA, ENC1_CHB, ENC2_CHA, ENC2_CHB, ENC3_CHA, ENC3_CHB, ENC4_CHA, ENC4_CHB;
reg         ENC1_SW, ENC3_SW, ENC4_SW;
reg         UART_RXD;

wire        ADC_SCLK;
wire        ADC_CS_N;
wire        ADC_MOSI;
wire [7:0]  LED;
wire        BUZZER;
wire        UART_TXD;
wire [2:0]  HDMI_TX_P;
wire        HDMI_CLK_P;
wire        I2C_SCL;
wire        I2C_SDA;
wire        WS2812_DIN;

parameter PERIOD_25M = 40;
always #(PERIOD_25M/2) SYSCLK_25M = ~SYSCLK_25M;

top #(
    .CLK_FREQ (100_000_000)
) uut (
    .SYSCLK_25M   (SYSCLK_25M),
    .ADC_SCLK     (ADC_SCLK),
    .ADC_CS_N     (ADC_CS_N),
    .ADC_MISO     (ADC_MISO),
    .ADC_MOSI     (ADC_MOSI),
    .HDMI_TX_P    (HDMI_TX_P),
    .HDMI_CLK_P   (HDMI_CLK_P),
    .LED          (LED),
    .BTN_SW2      (BTN_SW2),
    .BTN_SW3      (BTN_SW3),
    .BTN_FUNC     (BTN_FUNC),
    .BTN_VIEW     (BTN_VIEW),
    .BTN_FREEZE   (BTN_FREEZE),
    .BTN_SEL1     (BTN_SEL1),
    .BTN_SEL2     (BTN_SEL2),
    .BTN_SEL3     (BTN_SEL3),
    .ENC1_CHA       (ENC1_CHA),
    .ENC1_CHB       (ENC1_CHB),
    .ENC2_CHA       (ENC2_CHA),
    .ENC2_CHB       (ENC2_CHB),
    .ENC3_CHA       (ENC3_CHA),
    .ENC3_CHB       (ENC3_CHB),
    .ENC4_CHA       (ENC4_CHA),
    .ENC4_CHB       (ENC4_CHB),
    .WS2812_DIN   (WS2812_DIN),
    .BUZZER       (BUZZER),
    .UART_TXD     (UART_TXD),
    .UART_RXD     (UART_RXD),
    .I2C_SDA      (I2C_SDA),
    .I2C_SCL      (I2C_SCL)
);

reg [11:0] adc_sim_data [0:7];
integer    adc_ch_i;
initial begin
    for (adc_ch_i = 0; adc_ch_i < 8; adc_ch_i = adc_ch_i + 1) begin
        adc_sim_data[adc_ch_i] = 12'd2048;
    end
end

reg  [15:0] adc_resp_shift;
reg         adc_resp_active;
reg         prev_sclk_top;
reg         prev_cs_n_top;
reg  [2:0]  adc_resp_ch;

always @(posedge SYSCLK_25M) begin
    prev_sclk_top <= ADC_SCLK;
    prev_cs_n_top <= ADC_CS_N;

    if (prev_cs_n_top == 1'b1 && ADC_CS_N == 1'b0) begin
        adc_resp_active = 1'b1;
        adc_resp_ch = adc_resp_ch + 1'b1;
        adc_resp_shift = {4'b0000, adc_sim_data[adc_resp_ch]};
    end

    if (ADC_CS_N == 1'b0 && prev_sclk_top == 1'b0 && ADC_SCLK == 1'b1 && adc_resp_active) begin
        ADC_MISO = adc_resp_shift[15];
        adc_resp_shift = adc_resp_shift << 1;
    end

    if (ADC_CS_N == 1'b1) begin
        adc_resp_active = 1'b0;
        ADC_MISO = 1'b0;
    end
end

integer led_change_cnt;
reg [7:0] prev_led;

always @(posedge SYSCLK_25M) begin
    prev_led <= LED;
    if (LED !== prev_led) led_change_cnt = led_change_cnt + 1;
end

integer tmds_toggle_cnt;
reg prev_hdmi_clk_p;

always @(posedge SYSCLK_25M) begin
    prev_hdmi_clk_p <= HDMI_CLK_P;
    if (HDMI_CLK_P !== prev_hdmi_clk_p) tmds_toggle_cnt = tmds_toggle_cnt + 1;
end

initial begin
    SYSCLK_25M = 1'b0;
    ADC_MISO   = 1'b0;
    BTN_SW2    = 1'b0;
    BTN_SW3    = 1'b0;
    BTN_FUNC   = 1'b1;      // 按键板低有效，空闲为高
    BTN_VIEW   = 1'b1;
    BTN_FREEZE = 1'b1;
    BTN_SEL1   = 1'b1;
    BTN_SEL2   = 1'b1;
    BTN_SEL3   = 1'b1;
    ENC1_CHA=1'b1; ENC1_CHB=1'b1; ENC2_CHA=1'b1; ENC2_CHB=1'b1;
    ENC3_CHA=1'b1; ENC3_CHB=1'b1; ENC4_CHA=1'b1; ENC4_CHB=1'b1;
    ENC1_SW=1'b1; ENC3_SW=1'b1; ENC4_SW=1'b1;
    UART_RXD   = 1'b1;
    adc_resp_shift  = 16'd0;
    adc_resp_active = 1'b0;
    prev_sclk_top   = 1'b0;
    prev_cs_n_top   = 1'b1;
    adc_resp_ch     = 3'd0;
    led_change_cnt  = 0;
    prev_led        = 8'd0;
    tmds_toggle_cnt = 0;
    prev_hdmi_clk_p = 1'b0;

    #1000;

    $display("--- Test 1: DDS self-test mode (default) ---");
        #200_000_000;

    $display("After 200ms in DDS mode:");
    $display("  LED=%08b (bit6=DDS mode, bit7=rst_n, bit5=pll_locked)", LED);
    $display("  LED changes: %0d", led_change_cnt);
    $display("  HDMI CLK toggles: %0d", tmds_toggle_cnt);

    if (LED[6])
        $display("PASS: LED[6] on (DDS self-test mode active)");
    else
        $display("CHECK: LED[6] not on");

    if (LED[7])
        $display("PASS: LED[7] on (reset released)");
    else
        $display("CHECK: LED[7] not on (still in reset?)");

    if (tmds_toggle_cnt > 1000)
        $display("PASS: HDMI clock active");
    else
        $display("CHECK: HDMI clock not active (toggles=%0d)", tmds_toggle_cnt);

    $display("--- Test 2: ADC mode ---");
        led_change_cnt  = 0;
    tmds_toggle_cnt = 0;
    #100_000_000;

    $display("After 100ms in ADC mode:");
    $display("  LED=%08b", LED);
    $display("  LED changes: %0d", led_change_cnt);
    $display("  HDMI CLK toggles: %0d", tmds_toggle_cnt);

    $display("--- Test 3: Button press in DDS mode ---");
        #10_000_000;
    BTN_FUNC = 1'b0;             // 低有效，按 1.1ms
    #1_100_000;
    BTN_FUNC = 1'b1;
    #10_000_000;
    $display("After btn_func press: LED=%08b (bit6 应灭：离开自检)", LED);

    $display("--- Test 4: Encoder rotation ---");
    ENC1_CHA = 1'b0; #2000;        // 空闲 11 → CW: A↓B↓A↑B↑
    ENC1_CHB = 1'b0; #2000;
    ENC1_CHA = 1'b1; #2000;
    ENC1_CHB = 1'b1; #2000;
    #10_000_000;
    $display("After encoder step: LED=%08b", LED);

    $display("--- All top tests done ---");
    $finish;
end

initial begin
    #2_000_000_000;
    $display("TIMEOUT: top_tb");
    $finish;
end

endmodule