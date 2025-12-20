@echo off
echo ============================================
echo   370z Monitor - Backup to Google Drive
echo ============================================
echo.

:: SquareLine Project
echo [2/2] Syncing SquareLine project...
echo       Cleaning destination...
rd /s /q "G:\My Drive\Arduino\repos\SquareLine Studio\!370z Monitor" 2>nul
echo       Waiting for Google Drive to release...
timeout /t 5 /nobreak >nul
mkdir "G:\My Drive\Arduino\repos\SquareLine Studio\!370z Monitor" 2>nul
echo       Copying files...
robocopy "C:\source\370zMonitor\SquareLine" "G:\My Drive\Arduino\repos\SquareLine Studio\!370z Monitor" /MIR /XD .git /XF *.tmp /R:3 /W:5
if %ERRORLEVEL% LEQ 3 (
    echo       SquareLine sync complete!
) else (
    echo       WARNING: SquareLine sync had issues
)
echo.

echo ============================================
echo   Backup complete!
echo ============================================
pause