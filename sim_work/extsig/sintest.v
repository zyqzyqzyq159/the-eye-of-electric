`timescale 1ns / 1ps
module sintest;
integer i;
real t_us, ang;
initial begin
    for (i = 0; i < 8; i = i + 1) begin
        #14500;
        t_us = $realtime / 1000.0;
        ang  = 6.283185307179586 * 50.0 * t_us * 1.0e-3;
        $display("t=%0f ns  t_us=%0f  ang=%0f  sin=%0f  v=%0d",
                 $realtime, t_us, ang, $sin(ang), $rtoi(2100.0 + 819.0 * $sin(ang)));
    end
end
endmodule
