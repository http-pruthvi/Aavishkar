# V2V Hardware Deployment Guide

## 1. Hardware Setup
- **Raspberry Pi 4** running Ubuntu or Raspberry Pi OS.
- **USB GPS Module** plugged into any USB port.
- **Power Source** (Power bank or car USB).

## 2. Installation (On the Pi)
Copy the `v2v_realworld` folder to the Pi.

```bash
cd v2v_realworld
pip install -r requirements.txt
chmod +x setup_adhoc.sh
```

## 3. Network Configuration
Run this to create the mesh network. Assign a unique IP to each car.

**Vehicle 1:**
```bash
sudo ./setup_adhoc.sh 192.168.1.10
```

**Vehicle 2:**
```bash
sudo ./setup_adhoc.sh 192.168.1.11
```

## 4. Run the OBU Software
Start the software. Pass a unique Vehicle ID (integer).

**Vehicle 1:**
```bash
python3 gps_obu.py 1
```

**Vehicle 2:**
```bash
python3 gps_obu.py 2
```

## 5. Verification
- You should see "GPS Connected" (or "Simulation Mode" if GPS is missing).
- When both scripts are running, Vehicle 1 should see "Received BSM from Vehicle 2" and vice versa.
