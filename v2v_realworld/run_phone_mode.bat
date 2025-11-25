@echo off
echo Starting V2V System in PHONE GPS MODE...
echo.
echo 1. Make sure your Phone and Laptop are on the SAME WiFi network.
echo 2. Open your GPS App (e.g., GPS Output).
echo 3. Set the Target IP to your Laptop's IP (see below).
echo 4. Set the Port to 5555.
echo.
ipconfig | findstr "IPv4"
echo.
echo Launching software...
python gps_obu.py 1
pause
