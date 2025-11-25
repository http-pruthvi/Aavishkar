# How to use your Phone via USB

## 1. Enable USB Debugging
1.  On your Phone, go to **Settings > About Phone**.
2.  Tap **Build Number** 7 times to enable Developer Options.
3.  Go to **Settings > System > Developer Options**.
4.  Enable **USB Debugging**.

## 2. Install ADB (If needed)
If `run_usb_mode.bat` says "adb is not recognized":
1.  Download [SDK Platform Tools](https://developer.android.com/studio/releases/platform-tools) for Windows.
2.  Extract the folder.
3.  Copy `adb.exe`, `AdbWinApi.dll`, and `AdbWinUsbApi.dll` into this `v2v_realworld` folder.

## 3. Configure GPS App
1.  Open your GPS App (e.g., GPS Output).
2.  **Target IP**: `localhost` or `127.0.0.1`
3.  **Port**: `5555`
4.  **Protocol**: TCP (preferred for USB) or UDP.

## 4. Run It
Double-click `run_usb_mode.bat`.
-   Accept the "Allow USB Debugging?" prompt on your phone screen.
-   You should see GPS data flowing in the terminal.
