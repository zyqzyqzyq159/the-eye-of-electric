@echo off
REM ============================================================
REM  AL_TRY UI one-click build (build)
REM  Parameters are FROZEN here - do NOT rebuild from TD GUI:
REM  GUI default = medium effort, which ALWAYS fails PHY-8023
REM  on this design and corrupts data on the board.
REM  V25b lesson (keep ASCII-only comments in .bat - GBK cmd
REM  mangles UTF-8 Chinese into bogus commands): TD continues
REM  to bitgen even after PHY-9009 (mslice overflow) placement
REM  error, producing a BAD bitstream - so the error check must
REM  cover PHY-9xxx, not just PHY-8023.
REM ============================================================
cd /d %~dp0AL_TRY_Runs\build
if exist AL_TRY.bit del AL_TRY.bit
C:\Anlogic\TD_5.6.4_Release_97.693\bin\td.exe -nogui -b < flow.tcl > build_bat.log 2>&1
findstr /C:"PHY-8023" build_bat.log >nul
if %errorlevel%==0 (
    echo.
    echo BUILD FAILED: PHY-8023 routing violations found in build_bat.log
    echo Re-run this script, or change "relaxation" in flow.tcl ^(1.15/1.2/1.25^)
    exit /b 1
)
findstr /R /C:"PHY-9[0-9][0-9][0-9]" build_bat.log >nul
if %errorlevel%==0 (
    echo.
    echo BUILD FAILED: placement/route error ^(PHY-9xxx, e.g. PHY-9009 mslice
    echo overflow^) found in build_bat.log - bitstream is BAD, do NOT flash.
    echo Reduce logic ^(LUT/mslice^) and rebuild.
    exit /b 1
)
if not exist AL_TRY.bit (
    echo.
    echo BUILD FAILED: no bitstream generated, check build_bat.log
    exit /b 1
)
echo.
echo BUILD OK: AL_TRY_Runs\build\AL_TRY.bit
echo Download with: download_ui.bat  ^(connect the board first^)
