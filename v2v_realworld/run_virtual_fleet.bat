@echo off
echo Starting Virtual V2V Fleet...
echo.
echo Launching Vehicle 1...
start "Vehicle 1 (OBU)" cmd /k "python gps_obu.py 1"
timeout /t 2 >nul
echo Launching Vehicle 2...
start "Vehicle 2 (OBU)" cmd /k "python gps_obu.py 2"
echo.
echo Both vehicles are running. They will simulate movement and communicate over your local network.
echo Watch the windows for "RX" messages and Collision Warnings.
pause
