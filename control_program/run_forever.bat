@echo off
:loop

:: Run MAVP2P app connecting to the program's UDP Bridge (14445 and 14446)
:: instead of locking the COM port directly, allowing both programs to work concurrently.
start /wait mavp2p.exe udpc:127.0.0.1:19415 udpc:127.0.0.1:14556 udps:127.0.0.1:14446 udpc:127.0.0.1:14445

if %ERRORLEVEL% equ 0 (
  :: Stop script if exit code is "0"
  exit /b
)

:: Wait for 1 second before restarting (optional)
timeout /t 1 >nul

:: Restart MAVP2P on non-zero exit code
goto loop
