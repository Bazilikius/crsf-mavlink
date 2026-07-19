@echo off
:loop
:: Update "COM123" and baud rate as needed.
set serial_port=COM123:460800

:: Run MAVP2P app and wait for it to stop
start /wait mavp2p.exe udpc:127.0.0.1:19415 udpc:127.0.0.1:14556 serial:%serial_port%

if %ERRORLEVEL% equ 0 (
  :: Stop script if exit code is "0"
  exit /b
)

:: Wait for 1 second before restarting (optional)
timeout /t 1 >nul

:: Restart MAVP2P on non-zero exit code
goto loop
