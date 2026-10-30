@echo off
REM  AL_TRY UI one-click JTAG download (connect board, then run this)
cd /d %~dp0
C:\Anlogic\TD_5.6.4_Release_97.693\bin\td.exe -nogui -b < download_ui.tcl
