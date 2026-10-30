`timescale 1ns / 1ps
// user_input 行为仿真：按键板 6 键 + 4 编码器 + SW2/SW3 备份键
// 验证：功能/显示模式循环、定格、自检注入开关、测量情境键、
//       编码器四分之一计数制（4 沿 = 1 步）、视图参数更新与翻转握手
module user_input_tb;

reg clk;
reg rst_n;

reg btn_func, btn_view, btn_freeze, btn_sel1, btn_sel2, btn_sel3;
reg btn_sw2, btn_sw3;
reg enc1_a, enc1_b, enc2_a, enc2_b, enc3_a, enc3_b, enc4_a, enc4_b;
reg enc1_sw, enc3_sw, enc4_sw;

wire [1:0]  func_mode, disp_mode, meas_chan, meas_pick;
wire        freeze, meas_vi;
wire [2:0]  self_sel;
wire [10:0] func_data;
wire        func_toggle;
wire [44:0] view_data;
wire        view_toggle;
wire        mode_change_pulse;

parameter CLK_PERIOD = 10;         // 100MHz
parameter TQ = 200_000;            // 编码器正交跳变间隔 200µs

initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

user_input #(
    .CLK_FREQ(100_000_000), .DEBOUNCE_MS(1), .EXT_BTN_ACTIVE_LOW(1)
) uut (
    .clk(clk), .rst_n(rst_n),
    .btn_func(btn_func), .btn_view(btn_view), .btn_freeze(btn_freeze),
    .btn_sel1(btn_sel1), .btn_sel2(btn_sel2), .btn_sel3(btn_sel3),
    .btn_sw2(btn_sw2), .btn_sw3(btn_sw3),
    .enc1_a(enc1_a), .enc1_b(enc1_b), .enc1_sw(enc1_sw),
    .enc2_a(enc2_a), .enc2_b(enc2_b),
    .enc3_a(enc3_a), .enc3_b(enc3_b), .enc3_sw(enc3_sw),
    .enc4_a(enc4_a), .enc4_b(enc4_b), .enc4_sw(enc4_sw),
    .func_mode(func_mode), .disp_mode(disp_mode), .freeze(freeze),
    .self_sel(self_sel), .meas_chan(meas_chan), .meas_vi(meas_vi), .meas_pick(meas_pick),
    .func_data(func_data), .func_toggle(func_toggle),
    .view_data(view_data), .view_toggle(view_toggle),
    .mode_change_pulse(mode_change_pulse)
);

// 板外按键：低有效，按住 2.5ms（>1ms 消抖）
task press_ext(input integer which);
    begin
        case (which)
            0: begin btn_func = 1'b0;   #(CLK_PERIOD*150_000); btn_func = 1'b1;   end
            1: begin btn_view = 1'b0;   #(CLK_PERIOD*150_000); btn_view = 1'b1;   end
            2: begin btn_freeze = 1'b0; #(CLK_PERIOD*150_000); btn_freeze = 1'b1; end
            3: begin btn_sel1 = 1'b0;   #(CLK_PERIOD*150_000); btn_sel1 = 1'b1;   end
            4: begin btn_sel2 = 1'b0;   #(CLK_PERIOD*150_000); btn_sel2 = 1'b1;   end
            5: begin btn_sel3 = 1'b0;   #(CLK_PERIOD*150_000); btn_sel3 = 1'b1;   end
        endcase
        #(CLK_PERIOD*150_000);
    end
endtask

// 编码器按压复位键：低有效 1.5ms
task press_encsw(input integer k);
    begin
        case (k)
            1: begin enc1_sw = 1'b0; #(CLK_PERIOD*150_000); enc1_sw = 1'b1; end
            3: begin enc3_sw = 1'b0; #(CLK_PERIOD*150_000); enc3_sw = 1'b1; end
            4: begin enc4_sw = 1'b0; #(CLK_PERIOD*150_000); enc4_sw = 1'b1; end
        endcase
        #(CLK_PERIOD*150_000);
    end
endtask

// 核心板 SW2/SW3：高有效
task press_sw2;
    begin
        btn_sw2 = 1'b1; #(CLK_PERIOD*150_000); btn_sw2 = 1'b0; #(CLK_PERIOD*150_000);
    end
endtask
task press_sw3;
    begin
        btn_sw3 = 1'b1; #(CLK_PERIOD*150_000); btn_sw3 = 1'b0; #(CLK_PERIOD*150_000);
    end
endtask

// 编码器旋转一格：空闲 11，CW = A↓B↓A↑B↑（格雷 + 向），CCW = B↓A↓B↑A↑
task detent(input integer k, input cw);
    begin
        case (k)
            1: begin
                if (cw) begin enc1_a=0; #TQ; enc1_b=0; #TQ; enc1_a=1; #TQ; enc1_b=1; #TQ; end
                else    begin enc1_b=0; #TQ; enc1_a=0; #TQ; enc1_b=1; #TQ; enc1_a=1; #TQ; end
            end
            2: begin
                if (cw) begin enc2_a=0; #TQ; enc2_b=0; #TQ; enc2_a=1; #TQ; enc2_b=1; #TQ; end
                else    begin enc2_b=0; #TQ; enc2_a=0; #TQ; enc2_b=1; #TQ; enc2_a=1; #TQ; end
            end
            3: begin
                if (cw) begin enc3_a=0; #TQ; enc3_b=0; #TQ; enc3_a=1; #TQ; enc3_b=1; #TQ; end
                else    begin enc3_b=0; #TQ; enc3_a=0; #TQ; enc3_b=1; #TQ; enc3_a=1; #TQ; end
            end
            4: begin
                if (cw) begin enc4_a=0; #TQ; enc4_b=0; #TQ; enc4_a=1; #TQ; enc4_b=1; #TQ; end
                else    begin enc4_b=0; #TQ; enc4_a=0; #TQ; enc4_b=1; #TQ; enc4_a=1; #TQ; end
            end
        endcase
        #(CLK_PERIOD*100);
    end
endtask

reg ft_saved, vt_saved;
integer errors;


integer mc_cnt = 0;
always @(posedge clk) if (mode_change_pulse) mc_cnt = mc_cnt + 1;



initial begin
    rst_n = 1'b0;
    btn_func=1; btn_view=1; btn_freeze=1; btn_sel1=1; btn_sel2=1; btn_sel3=1;
    btn_sw2=0; btn_sw3=0;
    enc1_a=1; enc1_b=1; enc2_a=1; enc2_b=1; enc3_a=1; enc3_b=1; enc4_a=1; enc4_b=1;
    enc1_sw=1; enc3_sw=1; enc4_sw=1;
    errors = 0;

    #1000;
    rst_n = 1'b1;
    #(CLK_PERIOD*100);

    // --- Test 1: 复位默认 ---
    if (func_mode==0 && disp_mode==0 && freeze==0 && self_sel==0)
        $display("PASS: reset defaults (selftest/waveform/run)");
    else begin $display("FAIL: reset defaults %b %b %b %b", func_mode, disp_mode, freeze, self_sel); errors=errors+1; end

    // --- Test 2: FUNC 循环 0→1→2→3→0 + 翻转握手 ---
    ft_saved = func_toggle;
    press_ext(0);
    if (func_mode==1 && func_toggle!=ft_saved && func_data==11'b01_00_00_000_00)
        $display("PASS: FUNC -> 1PH-A, toggle fired, data=%02h", func_data);
    else begin $display("FAIL: FUNC press1 mode=%b data=%02h", func_mode, func_data); errors=errors+1; end
    press_ext(0); press_ext(0); press_ext(0);
    if (func_mode==0)
        $display("PASS: FUNC wraps 3 -> 0");
    else begin $display("FAIL: FUNC wrap mode=%b", func_mode); errors=errors+1; end

    // --- Test 3: VIEW 循环 + 脉冲 ---
    mc_cnt = 0;
    press_ext(1);
    if (disp_mode==1 && mc_cnt==1)
        $display("PASS: VIEW -> spectrum with pulse");
    else begin $display("FAIL: VIEW press1 disp=%b mc=%0d", disp_mode, mc_cnt); errors=errors+1; end
    #(CLK_PERIOD*10);
    press_ext(1);
    if (disp_mode==2) $display("PASS: VIEW -> harmonic table");
    else begin $display("FAIL: VIEW press2 disp=%b", disp_mode); errors=errors+1; end
    #(CLK_PERIOD*10);
    press_ext(1);
    if (disp_mode==0) $display("PASS: VIEW wraps to waveform (3-state cycle)");
    else begin $display("FAIL: VIEW wrap disp=%b", disp_mode); errors=errors+1; end
    #(CLK_PERIOD*10);

    // SW2 备份键等效 VIEW
    press_sw2;
    if (disp_mode==1) $display("PASS: SW2 backup key = VIEW");
    else begin $display("FAIL: SW2 backup disp=%b", disp_mode); errors=errors+1; end
    #(CLK_PERIOD*10);
    press_ext(1); press_ext(1); press_ext(1);   // 回到波形（1→2→3→0）

    // --- Test 4: FREEZE ---
    press_ext(2);
    if (freeze==1) $display("PASS: FREEZE on");
    else begin $display("FAIL: freeze=%b", freeze); errors=errors+1; end
    press_ext(2);
    if (freeze==0) $display("PASS: FREEZE off");
    else begin $display("FAIL: freeze=%b", freeze); errors=errors+1; end
    press_sw3;
    if (freeze==1) $display("PASS: SW3 backup key toggles freeze");
    else begin $display("FAIL: SW3 freeze=%b", freeze); errors=errors+1; end
    press_sw3;

    // --- Test 5: 自检模式 SEL1/2/3 = 注入开关 ---
    if (func_mode==0) begin
        press_ext(3); press_ext(4); press_ext(5);
        if (self_sel==3'b111)
            $display("PASS: selftest SEL1/2/3 -> self_sel=111");
        else begin $display("FAIL: self_sel=%b", self_sel); errors=errors+1; end
        press_ext(3); press_ext(4); press_ext(5);
        if (self_sel==3'b000) $display("PASS: selftest SEL toggles back");
        else begin $display("FAIL: self_sel=%b", self_sel); errors=errors+1; end
    end

    // --- Test 6: 测量模式（FUNC→单相）情境键 ---
    press_ext(0);   // func_mode=1
    if (func_mode==1) $display("PASS: enter 1PH-A mode");
    else begin $display("FAIL: func_mode=%b", func_mode); errors=errors+1; end
    press_ext(3);   // 通道循环 全显→A
    if (meas_chan==1) $display("PASS: SEL1 chan -> A");
    else begin $display("FAIL: meas_chan=%b", meas_chan); errors=errors+1; end
    press_ext(3); press_ext(3); press_ext(3);
    if (meas_chan==0) $display("PASS: SEL1 chan wraps to all");
    else begin $display("FAIL: meas_chan wrap=%b", meas_chan); errors=errors+1; end
    press_ext(4);
    if (meas_vi==1) $display("PASS: SEL2 -> current");
    else begin $display("FAIL: meas_vi=%b", meas_vi); errors=errors+1; end
    press_ext(4);
    press_ext(5);
    if (meas_pick==1) $display("PASS: SEL3 pick -> A");
    else begin $display("FAIL: meas_pick=%b", meas_pick); errors=errors+1; end
    press_ext(5); press_ext(5); press_ext(5);   // pick 1→2→3→0 回全显

    // --- Test 7: 编码器1 时基（4 沿 = 1 步）---
    vt_saved = view_toggle;
    detent(1, 1);
    if (view_toggle!=vt_saved && $signed(view_data[44:42])==1)
        $display("PASS: ENC1 CW -> timebase +1 (view_data tb=%0d)", $signed(view_data[44:42]));
    else begin $display("FAIL: ENC1 CW tb=%0d vt=%b", $signed(view_data[44:42]), view_toggle); errors=errors+1; end
    detent(1, 0); detent(1, 0);
    if ($signed(view_data[44:42])==-1)
        $display("PASS: ENC1 CCW x2 -> timebase clamped -1");
    else begin $display("FAIL: tb clamp=%0d", $signed(view_data[44:42])); errors=errors+1; end
    detent(1, 0);
    if ($signed(view_data[44:42])==-1)
        $display("PASS: ENC1 CCW at min stays -1");
    else begin $display("FAIL: tb min=%0d", $signed(view_data[44:42])); errors=errors+1; end
    detent(1, 1);
    if ($signed(view_data[44:42])==0) $display("PASS: ENC1 back to 0");
    else begin $display("FAIL: tb=%0d", $signed(view_data[44:42])); errors=errors+1; end

    // --- Test 8: 编码器2 平移 ---
    vt_saved = view_toggle;
    detent(2, 1);
    if (view_toggle!=vt_saved && view_data[41:33]==9'd8)
        $display("PASS: ENC2 CW -> pan +8");
    else begin $display("FAIL: pan=%0d", view_data[41:33]); errors=errors+1; end

    // --- Test 9: 编码器3/4 垂直位置/缩放（pick=0 作用于全部窗口）---
    detent(3, 1);
    if ($signed(view_data[32:25])==8 && $signed(view_data[24:17])==8 && $signed(view_data[16:9])==8)
        $display("PASS: ENC3 CW -> all windows vpos +8");
    else begin $display("FAIL: vpos a=%0d b=%0d c=%0d", $signed(view_data[32:25]), $signed(view_data[24:17]), $signed(view_data[16:9])); errors=errors+1; end
    detent(4, 1);
    if ($signed(view_data[8:6])==1 && $signed(view_data[5:3])==1 && $signed(view_data[2:0])==1)
        $display("PASS: ENC4 CW -> all windows scale offset +1");
    else begin $display("FAIL: soff a=%0d b=%0d c=%0d", $signed(view_data[8:6]), $signed(view_data[5:3]), $signed(view_data[2:0])); errors=errors+1; end

    // --- Test 10: pick=A 后只作用于 A 窗 ---
    press_ext(5);   // pick=1（当前 func=1 测量模式）
    if (meas_pick==1) begin
        detent(3, 1);
        if ($signed(view_data[32:25])==16 && $signed(view_data[24:17])==8 && $signed(view_data[16:9])==8)
            $display("PASS: pick=A -> only A vpos moves (+16 vs +8)");
        else begin $display("FAIL: vpos a=%0d b=%0d c=%0d", $signed(view_data[32:25]), $signed(view_data[24:17]), $signed(view_data[16:9])); errors=errors+1; end
    end

    press_ext(5); press_ext(5); press_ext(5);   // pick 1→2→3→0 回全显
    press_encsw(3);                              // vpos 全窗回 0（顺带首验复位功能）

    // --- Test 11: 编码器抖动（单次跳变来回）不产生步进 ---
    vt_saved = view_toggle;
    enc1_a = 0; #(CLK_PERIOD*100); enc1_a = 1; #(CLK_PERIOD*100);   // 单沿抖动
    if (view_toggle==vt_saved && $signed(view_data[44:42])==0)
        $display("PASS: contact glitch produces no step");
    else begin $display("FAIL: glitch produced toggle/data change"); errors=errors+1; end

    // --- Test 12: 编码器按压复位（把参数调偏后逐个按压）---
    detent(1, 1); detent(1, 1);                      // tb 0→+2
    detent(3, 1); detent(3, 1);                      // vpos 全部 +16
    detent(4, 1); detent(4, 1);                      // soff 全部 +3（测试 9 已 +1，且 +3 为上限）
    if ($signed(view_data[44:42])==2 && $signed(view_data[32:25])==16 && $signed(view_data[8:6])==3)
        $display("PASS: params adjusted away from defaults");
    else begin $display("FAIL: pre-reset state tb=%0d vpos=%0d soff=%0d",
        $signed(view_data[44:42]), $signed(view_data[32:25]), $signed(view_data[8:6])); errors=errors+1; end

    press_encsw(1);
    if ($signed(view_data[44:42])==0 && $signed(view_data[32:25])==16)
        $display("PASS: ENC1_SW resets timebase only");
    else begin $display("FAIL: after ENC1_SW tb=%0d vpos=%0d", $signed(view_data[44:42]), $signed(view_data[32:25])); errors=errors+1; end

    press_encsw(4);
    if ($signed(view_data[8:6])==0 && $signed(view_data[5:3])==0 && $signed(view_data[2:0])==0 && $signed(view_data[32:25])==16)
        $display("PASS: ENC4_SW resets all scale offsets (back to auto)");
    else begin $display("FAIL: after ENC4_SW soff=%0d/%0d/%0d vpos=%0d",
        $signed(view_data[8:6]), $signed(view_data[5:3]), $signed(view_data[2:0]), $signed(view_data[32:25])); errors=errors+1; end

    press_encsw(3);
    if ($signed(view_data[32:25])==0 && $signed(view_data[24:17])==0 && $signed(view_data[16:9])==0)
        $display("PASS: ENC3_SW resets all vpos to 0");
    else begin $display("FAIL: after ENC3_SW vpos=%0d/%0d/%0d",
        $signed(view_data[32:25]), $signed(view_data[24:17]), $signed(view_data[16:9])); errors=errors+1; end

    if (errors == 0)
        $display("=== user_input_tb ALL PASS ===");
    else
        $display("=== user_input_tb %0d FAILURES ===", errors);
    $finish;
end

initial begin
    #250_000_000;
    $display("TIMEOUT: user_input_tb");
    $finish;
end

endmodule
