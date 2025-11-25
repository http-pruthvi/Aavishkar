@echo off
echo Starting V2V System in USB MODE...
echo.
echo Checking for ADB device...
adb devices
echo.
echo Setting up Port Forwarding (Phone 5555 -> Laptop 5555)...
adb forward tcp:5555 tcp:5555
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Could not setup ADB forwarding.
    echo 1. Is your phone connected?
    echo 2. Is USB Debugging enabled?
    echo 3. Do you have 'adb' installed? (If not, download Platform Tools)
    pause
    exit /b
)
echo.
echo Forwarding Active. Launching Software...
python gps_obu.py 1
pause
