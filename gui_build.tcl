# =====================================================================
#  在 TD GUI 里编译本项目（2026-10-20）
#  用法：打开 TD 任意界面 → Tcl 命令行（Tools/View 菜单里找 Tcl Console
#        或底部命令行）→ 粘贴执行这一行：
#            source C:/Users/hp/Desktop/FPGA/AL_TRY/gui_build.tcl
#  等价于 build_ui.bat（冻结参数：place effort/timing high + relaxation 1.2
#  + cn_opt max + route effort/opt_timing high）——GUI 默认 medium effort
#  在本设计（84% LUT）必出 PHY-8023，勿用默认 Run 直接编。
#  产物：AL_TRY_Runs/build/AL_TRY.bit；若日志出现 PHY-8023 则位流作废。
# =====================================================================
cd {C:/Users/hp/Desktop/FPGA/AL_TRY/AL_TRY_Runs/build}
open_project {AL_TRY.prj}
import_device eagle_s20.db -package EG4S20NG88
elaborate -top {top}
read_adc ../../uisrc/04_pin/top.adc
optimize_rtl
read_sdc ../../uisrc/04_pin/top.sdc
optimize_gate
legalize_phy_inst
set_param place effort high
set_param place timing high
set_param place cn_opt_effort max
set_param place relaxation 1.2
set_param route effort high
set_param route opt_timing high
place
route
report_timing -mode FINAL -file ui_timing.rpt
report_area -io_info -file ui_area.rpt
export_bid AL_TRY_inst.bid
bitgen -bit AL_TRY.bit
puts "=================================================================="
puts " GUI BUILD DONE. bitstream: C:/Users/hp/Desktop/FPGA/AL_TRY/AL_TRY_Runs/build/AL_TRY.bit"
puts " 若上方日志出现 PHY-8023 则位流作废（重跑本脚本即可）"
puts "=================================================================="
