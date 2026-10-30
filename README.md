# 电眸 —— 基于安路 EG4S20 的三相电能质量掌上分析仪

以 Anlogic EG4S20 FPGA 为核心的三相电能质量分析仪器，全部数字逻辑以 Verilog 实现。

## 功能

- 三相电压/电流采集，真有效值（RMS）与频率测量，PLL 同步锁相
- FFT 谐波分析：THD（总谐波失真）、3/5 次谐波、三相不平衡度
- TFT 彩屏界面：波形迹线（A 相 2 倍上采样插值圆滑）、谐波频谱、测量读数、中文屏幕标签（点阵字库 ROM）
- 测量/自检双模式，无信号自动判别（电压 <10V / 电流 <0.5A 时读数置零）
- 事件检测、UART 数据上传（VOFA+）、WS2812 状态指示、按键/编码器交互

## 工程结构

```
AL_TRY.al        安路 TD 工程文件
uisrc/01_rtl     RTL 源码；gen_*.py 为 ROM 初始化数据生成脚本（Python + PIL）
uisrc/02_sim     显示模块回归仿真（ModelSim）
uisrc/03_ip      TD IP 配置
uisrc/04_pin     引脚约束
uisrc/06_doc     文档
sim_check/       仿真检查
sim_work/        外部激励回归（extsig）
```

## 构建与下载

- 用 Anlogic TD 打开 `AL_TRY.al` 工程编译，或直接运行 `build_ui.bat`
- `download_ui.bat` 一键下载比特流
- 回归仿真：ModelSim，见 `uisrc/02_sim` 与 `sim_work/extsig`

## 工具链

Anlogic TD · ModelSim · Python (PIL)

## 资源占用

LUT 约 90%（EG4S20）
