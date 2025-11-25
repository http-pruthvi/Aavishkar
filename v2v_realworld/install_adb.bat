@echo off
echo Downloading Android Platform Tools (ADB)...
echo.

set DOWNLOAD_URL=https://dl.google.com/android/repository/platform-tools-latest-windows.zip
set DOWNLOAD_FILE=platform-tools.zip

echo Downloading from: %DOWNLOAD_URL%
powershell -Command "Invoke-WebRequest -Uri '%DOWNLOAD_URL%' -OutFile '%DOWNLOAD_FILE%'"

if not exist %DOWNLOAD_FILE% (
    echo Download failed. Please download manually from:
    echo https://developer.android.com/studio/releases/platform-tools
    pause
    exit /b
)

echo.
echo Extracting...
powershell -Command "Expand-Archive -Path '%DOWNLOAD_FILE%' -DestinationPath '.' -Force"

echo.
echo Copying ADB files to current directory...
copy platform-tools\adb.exe .
copy platform-tools\AdbWinApi.dll .
copy platform-tools\AdbWinUsbApi.dll .

echo.
echo Cleaning up...
del %DOWNLOAD_FILE%
rmdir /s /q platform-tools

echo.
echo ADB installed successfully!
echo You can now run run_usb_mode.bat
pause
