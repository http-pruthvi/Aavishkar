# How to use your Phone as a V2V Sensor

## 1. Get the App
Download a free app that streams NMEA data via UDP.
- **Android**: "GPS Output" or "SensorStream"
- **iOS**: "Sensorlog" or similar NMEA streaming apps.

## 2. Connect to WiFi
Ensure your Laptop and Phone are connected to the **same WiFi network**.
*(Tip: You can also turn on your Phone's Hotspot and connect your Laptop to it).*

## 3. Configure the App
1. Open the App.
2. Look for **UDP Stream** settings.
3. **Target IP**: Enter your Laptop's IP Address.
   - Run `run_phone_mode.bat` to see your IP.
4. **Port**: Set to `5555`.
5. **Start Streaming**.

## 4. Run the System
Double-click `run_phone_mode.bat`.
You should see:
```
📱 Waiting for Phone GPS data on UDP port 5555...
📍 Phone GPS: 12.9723, 77.5932 | Speed: 15.4
```
This confirms your phone is driving the V2V system!
