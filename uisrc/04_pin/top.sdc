create_clock -name SYS_CLK -period 40 -waveform {0 20} [get_ports {SYSCLK_25M}]

# 自动在两个 PLL 的输出端口生成时钟约束（100M / 25M / 75M / 375M），
# 频率相位按 PLL 参数设定。此前仅约束 25M 输入，100M FFT 域与 75M
# 显示域完全未被 STA 分析。
derive_pll_clocks

# 两个 PLL 输出为异步域（跨域均经翻转握手/同步器），不分析跨组路径。
# 注意：TD 的 get_clocks 通配符不跨层次（pll_sys* 匹配不到完整名），
# 且 read_sdc 不支持 Tcl 变量赋值，只能用完整时钟名逐条展开。
# pll_hdmi 三路输出同源同步，互不设异步；仅设 pll_sys 与各路间异步。
set_clock_groups -asynchronous \
    -group [get_clocks {pll_sys_inst/pll_inst.clkc[0]}] \
    -group [get_clocks {pll_hdmi_inst/pll_inst.clkc[0]}]
set_clock_groups -asynchronous \
    -group [get_clocks {pll_sys_inst/pll_inst.clkc[0]}] \
    -group [get_clocks {pll_hdmi_inst/pll_inst.clkc[1]}]
set_clock_groups -asynchronous \
    -group [get_clocks {pll_sys_inst/pll_inst.clkc[0]}] \
    -group [get_clocks {pll_hdmi_inst/pll_inst.clkc[2]}]

# HDMI 为源同步 LVDS 输出：数据与随行时钟由 IO 电气特性与 PCB 等长保证，
# 375M 串行域寄存器到引脚的路径不做 setup 分析（虚假违例）。
set_false_path -to [get_ports HDMI_TX_P*]
set_false_path -to [get_ports HDMI_CLK_P]
