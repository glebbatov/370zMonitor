@echo off
echo ============================================
echo   370z Monitor - Backup to Google Drive
echo ============================================
echo.

:: Arduino Project
echo [1/2] Syncing Arduino project...
echo       Cleaning destination...
rd /s /q "G:\My Drive\Arduino\repos\!CAN\370zMonitor" 2>nul
echo       Waiting for Google Drive to release...
timeout /t 5 /nobreak >nul
mkdir "G:\My Drive\Arduino\repos\!CAN\370zMonitor" 2>nul
echo       Copying files...
robocopy "C:\source\370zMonitor\Arduino" "G:\My Drive\Arduino\repos\!CAN\370zMonitor" /MIR /XD .git build /XF *.elf *.bin *.o /R:3 /W:5
if %ERRORLEVEL% LEQ 3 (
    echo       Arduino sync complete!
) else (
    echo       WARNING: Arduino sync had issues
)
echo.

echo ============================================
echo   Backup complete!
echo ============================================
pause