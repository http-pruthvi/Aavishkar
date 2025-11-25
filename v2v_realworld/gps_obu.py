import serial
import time
import pynmea2
import json
import socket
import threading
import math

# Configuration
# Configuration
# Try to find a valid serial port, or default to None
GPS_PORT = 'COM3' # Example for Windows, will failover to Simulation
UDP_GPS_PORT = 5555 # Port to listen for Phone GPS
BAUD_RATE = 9600
BROADCAST_PORT = 50000 # High port to avoid conflicts

def get_broadcast_address():
    try:
        # Connect to an external server (doesn't send data) to get local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        # Assume /24 subnet for simplicity
        return '.'.join(local_ip.split('.')[:-1]) + '.255'
    except:
        return '255.255.255.255'

BROADCAST_IP = get_broadcast_address()

class V2V_OBU:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.running = True
        
        # State
        self.lat = 0.0
        self.lon = 0.0
        self.speed = 0.0 # km/h
        self.heading = 0.0

    def read_gps(self):
        print(f"Connecting to GPS at {GPS_PORT}...")
        try:
            ser = serial.Serial(GPS_PORT, BAUD_RATE, timeout=1)
            print("GPS Connected.")
        except Exception as e:
            print(f"Error connecting to GPS: {e}")
            print("Running in SIMULATION MODE (Fake GPS)")
            self.run_simulation_gps()
            return

        while self.running:
            try:
                line = ser.readline().decode('ascii', errors='replace')
                if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    msg = pynmea2.parse(line)
                    self.lat = msg.latitude
                    self.lon = msg.longitude
                    self.speed = msg.spd_over_grnd * 1.852 if msg.spd_over_grnd else 0.0 # Knots to km/h
                    self.heading = msg.true_course if msg.true_course else 0.0
            except Exception as e:
                pass # Ignore parse errors

    def read_udp_gps(self):
        """Listens for NMEA strings from Smartphone via UDP"""
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.bind(('0.0.0.0', UDP_GPS_PORT))
        print(f"📱 Waiting for Phone GPS data on UDP port {UDP_GPS_PORT}...")
        print(f"   (Configure your phone app to send to this laptop's IP)")

        while self.running:
            try:
                data, addr = udp_sock.recvfrom(1024)
                line = data.decode('ascii', errors='replace').strip()
                # Some apps send multiple lines
                for sentence in line.split('\n'):
                    if sentence.startswith('$G'):
                        msg = pynmea2.parse(sentence)
                        self.lat = msg.latitude
                        self.lon = msg.longitude
                        if hasattr(msg, 'spd_over_grnd'):
                            self.speed = float(msg.spd_over_grnd) * 1.852
                        if hasattr(msg, 'true_course'):
                            self.heading = float(msg.true_course)
                        
                        # Print status occasionally
                        if int(time.time()) % 2 == 0:
                            print(f"📍 Phone GPS: {self.lat:.5f}, {self.lon:.5f} | Speed: {self.speed:.1f}")
            except Exception as e:
                pass

    def run_simulation_gps(self):
        """Fallback if no hardware is present: Simulates driving in a circle"""
        import math
        print(f"⚠️  NO GPS HARDWARE FOUND. Running in VIRTUAL MODE.")
        print(f"🚗  Vehicle {self.vehicle_id} is driving in a virtual circle.")
        
        # Different starting angles for different vehicles so they don't overlap perfectly
        angle = self.vehicle_id * 3.14 
        center_lat = 12.9716
        center_lon = 77.5946
        radius = 0.0005 # approx 50m
        
        while self.running:
            # Move in a circle
            self.lat = center_lat + radius * math.cos(angle)
            self.lon = center_lon + radius * math.sin(angle)
            
            # Calculate heading (tangent to circle)
            self.heading = (math.degrees(angle) + 90) % 360
            self.speed = 30.0 # km/h
            
            angle += 0.1 # Move forward
            time.sleep(0.1)

    def broadcast_bsm(self):
        """Broadcasts Basic Safety Message 10 times per second"""
        print("Started BSM Broadcast...")
        while self.running:
            bsm = {
                'id': self.vehicle_id,
                'lat': self.lat,
                'lon': self.lon,
                'spd': self.speed,
                'hdg': self.heading,
                'ts': time.time()
            }
            data = json.dumps(bsm).encode('utf-8')
            self.sock.sendto(data, (BROADCAST_IP, BROADCAST_PORT))
            time.sleep(0.1) # 10 Hz

    def listen_for_peers(self):
        """Listens for other vehicles"""
        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.bind(('0.0.0.0', BROADCAST_PORT))
        print("Listening for V2V messages...")
        
        while self.running:
            data, addr = listener.recvfrom(1024)
            try:
                msg = json.loads(data.decode('utf-8'))
                if msg['id'] == self.vehicle_id: continue
                
                print(f"[{time.strftime('%H:%M:%S')}] 📡 RX from ID {msg['id']}: Dist {dist:.1f}m | Speed {msg['spd']:.0f} km/h")
                
                if dist < 20.0: # 20 meters
                    print(f"❌❌ CRITICAL COLLISION WARNING WITH VEHICLE {msg['id']} ❌❌")
                
            except Exception as e:
                # print(e)
                pass

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Haversine approximation
        R = 6371000 # Earth radius in meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def start(self):
        # Try Serial first, if fails, start UDP listener (Phone Mode)
        # For this hybrid version, we start UDP listener always
        t_udp = threading.Thread(target=self.read_udp_gps)
        t_udp.start()
        
        # We still try serial, but if it fails it falls back to sim. 
        # We want to avoid Sim if UDP is active. 
        # So we modify the logic slightly:
        t1 = threading.Thread(target=self.read_gps) # This will trigger Sim if Serial fails
        
        # NOTE: In a perfect world we'd have a clean state machine here.
        # For now, if you use Phone, ignore the "Virtual Mode" logs from the serial thread.
        
        t2 = threading.Thread(target=self.broadcast_bsm)
        t3 = threading.Thread(target=self.listen_for_peers)
        
        t1.start()
        t2.start()
        t3.start()
        
        try:
            while True: time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            print("Shutting down...")

if __name__ == "__main__":
    # In a real deployment, read ID from config
    import sys
    vid = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    
    obu = V2V_OBU(vid)
    obu.start()
