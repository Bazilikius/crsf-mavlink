@echo off
setlocal enabledelayedexpansion

:: Windows Batch script to launch mavp2p with auto-restart for RP2040 MAVLink bridge

:: Set your RP2040 COM port and baud rate here (e.g., start_mavp2p.bat COM80 115200)
set PORT=%1
if "%PORT%"=="" set PORT=COM80

set BAUD=%2
if "%BAUD%"=="" set BAUD=115200

set SERIAL_PORT=%PORT%:%BAUD%

:: Optional extra mavp2p flags, e.g.:  start_mavp2p.bat COM80 115200 --print-errors
set EXTRA=%3 %4 %5

echo --------------------------------------------------
echo   RP2040 MAVLink Bridge Runner (mavp2p)
echo   Serial Port : %SERIAL_PORT%
echo   UDP Servers : udps:127.0.0.1:19415
echo   UDP Client  : udpc:127.0.0.1:14556
echo --------------------------------------------------

:loop
echo [%TIME%] Starting mavp2p.exe...

start /wait mavp2p.exe %EXTRA% udps:127.0.0.1:19415 udpc:127.0.0.1:14556 serial:%SERIAL_PORT%

if %ERRORLEVEL% equ 0 (
  echo [%TIME%] mavp2p exited cleanly.
  exit /b
)

echo [%TIME%] mavp2p process stopped or disconnected. Reconnecting in 1 second...
timeout /t 1 >nul

goto loop
