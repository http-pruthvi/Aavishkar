#!/bin/bash

# Configuration
IFACE="wlan0"
IP_ADDR=$1 # Pass IP as argument (e.g., 192.168.1.10)

if [ -z "$IP_ADDR" ]; then
    echo "Usage: ./setup_adhoc.sh <IP_ADDRESS>"
    echo "Example: ./setup_adhoc.sh 192.168.1.10"
    exit 1
fi

echo "Configuring $IFACE for Ad-Hoc V2V Network..."

# Stop standard network managers to prevent interference
sudo systemctl stop NetworkManager
sudo systemctl stop wpa_supplicant

# Bring interface down
sudo ip link set $IFACE down

# Set Ad-Hoc mode (IBSS)
# Frequency 2412 MHz = Channel 1
sudo iwconfig $IFACE mode ad-hoc
sudo iwconfig $IFACE essid "V2V-NET"
sudo iwconfig $IFACE channel 1
sudo iwconfig $IFACE key off

# Set IP address
sudo ip addr flush dev $IFACE
sudo ip addr add $IP_ADDR/24 dev $IFACE

# Bring interface up
sudo ip link set $IFACE up

echo "Network configured. IP: $IP_ADDR"
echo "Ready for V2V communication."
