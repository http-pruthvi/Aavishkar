import tkinter as tk
from tkinter import ttk
import math
import time
import random

# --- Core Algorithms (The "Brain" of the System) ---

class KalmanFilter:
    """
    Simple 4-state Kalman Filter for tracking vehicle state [x, y, vx, vy].
    """
    def __init__(self, x=0, y=0, vx=0, vy=0):
        self.state = [x, y, vx, vy]
        self.P = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]] # Covariance

    def predict(self, dt):
        # F matrix (State Transition)
        # x = x + vx*dt
        # y = y + vy*dt
        self.state[0] += self.state[2] * dt
        self.state[1] += self.state[3] * dt
        # P = FPF' + Q (Simplified)

    def update(self, meas_x, meas_y, meas_vx, meas_vy):
        # Simplified update step for demonstration
        alpha = 0.8 # Trust measurement heavily
        self.state[0] = self.state[0] * (1-alpha) + meas_x * alpha
        self.state[1] = self.state[1] * (1-alpha) + meas_y * alpha
        self.state[2] = self.state[2] * (1-alpha) + meas_vx * alpha
        self.state[3] = self.state[3] * (1-alpha) + meas_vy * alpha

def calculate_ttc(ego_state, other_state):
    """
    Calculates Time-to-Collision (TTC) between two vehicles.
    """
    rx = other_state[0] - ego_state[0]
    ry = other_state[1] - ego_state[1]
    rvx = other_state[2] - ego_state[2]
    rvy = other_state[3] - ego_state[3]

    dot_r_v = rx * rvx + ry * rvy
    v_sq = rvx * rvx + rvy * rvy

    if v_sq < 1e-6:
        return float('inf')

    t = -dot_r_v / v_sq
    
    if t < 0:
        return float('inf') # Moving away

    # Check distance at closest approach
    dist_sq = (rx + rvx*t)**2 + (ry + rvy*t)**2
    
    # Collision radius (meters)
    if dist_sq < 16.0: # 4m radius squared
        return t
    
    return float('inf')

# --- Simulation Entities ---

class Vehicle:
    def __init__(self, id, x, y, speed, heading, color):
        self.id = id
        self.x = x
        self.y = y
        self.speed = speed # m/s
        self.heading = heading # radians
        self.color = color
        self.kf = KalmanFilter(x, y, speed*math.cos(heading), speed*math.sin(heading))
        self.braking = False

    def update(self, dt):
        if self.braking:
            self.speed = max(0, self.speed - 10 * dt) # Decelerate 10 m/s^2

        vx = self.speed * math.cos(self.heading)
        vy = self.speed * math.sin(self.heading)
        
        self.x += vx * dt
        self.y += vy * dt
        
        # Update internal estimation
        self.kf.predict(dt)
        self.kf.update(self.x, self.y, vx, vy)

    def get_bsm(self):
        """Broadcast Basic Safety Message"""
        return {
            'id': self.id,
            'x': self.x,
            'y': self.y,
            'vx': self.speed * math.cos(self.heading),
            'vy': self.speed * math.sin(self.heading),
            'speed': self.speed,
            'heading': self.heading
        }

# --- GUI Application ---

class V2VSimulator(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("V2V Collision Avoidance System - Software Demo")
        self.geometry("1000x700")
        self.configure(bg="#1e1e1e")

        # --- Dashboard ---
        self.dashboard = tk.Frame(self, bg="#2d2d2d", height=150)
        self.dashboard.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        self.lbl_status = tk.Label(self.dashboard, text="SYSTEM ACTIVE", font=("Segoe UI", 24, "bold"), bg="#2d2d2d", fg="#00ff00")
        self.lbl_status.pack(pady=10)
        
        self.lbl_info = tk.Label(self.dashboard, text="Monitoring V2V signals...", font=("Consolas", 14), bg="#2d2d2d", fg="white")
        self.lbl_info.pack()

        # --- Canvas ---
        self.canvas = tk.Canvas(self, bg="#121212", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # --- Controls ---
        self.controls = tk.Frame(self, bg="#1e1e1e")
        self.controls.pack(side=tk.BOTTOM, fill=tk.X, pady=10)
        
        ttk.Button(self.controls, text="Scenario 1: Intersection", command=self.scenario_intersection).pack(side=tk.LEFT, padx=10)
        ttk.Button(self.controls, text="Scenario 2: Rear End", command=self.scenario_rear_end).pack(side=tk.LEFT, padx=10)
        ttk.Button(self.controls, text="Scenario 3: Safe Pass", command=self.scenario_safe).pack(side=tk.LEFT, padx=10)
        ttk.Button(self.controls, text="Reset", command=self.reset).pack(side=tk.LEFT, padx=10)

        self.vehicles = []
        self.running = True
        self.last_time = time.time()
        
        self.reset()
        self.animate()

    def reset(self):
        self.vehicles = []
        self.lbl_status.config(text="SYSTEM ACTIVE", fg="#00ff00")
        self.canvas.delete("all")
        # Draw roads
        w = 1000
        h = 500
        # Horizontal road
        self.canvas.create_rectangle(0, h/2-40, w, h/2+40, fill="#333333", outline="")
        self.canvas.create_line(0, h/2, w, h/2, fill="white", dash=(20, 20))
        # Vertical road
        self.canvas.create_rectangle(w/2-40, 0, w/2+40, h, fill="#333333", outline="")
        self.canvas.create_line(w/2, 0, w/2, h, fill="white", dash=(20, 20))

    def scenario_intersection(self):
        self.reset()
        # Car 1 (Ego) moving Right
        self.vehicles.append(Vehicle(1, 100, 250+20, 15, 0, "#00aaff")) # Blue
        # Car 2 (Threat) moving Up
        self.vehicles.append(Vehicle(2, 500-20, 600, 15, -math.pi/2, "#ff4444")) # Red

    def scenario_rear_end(self):
        self.reset()
        # Car 1 (Ego) moving Right fast
        self.vehicles.append(Vehicle(1, 100, 250+20, 25, 0, "#00aaff"))
        # Car 2 (Threat) moving Right slow
        self.vehicles.append(Vehicle(2, 400, 250+20, 10, 0, "#ff4444"))

    def scenario_safe(self):
        self.reset()
        # Car 1 (Ego) moving Right
        self.vehicles.append(Vehicle(1, 100, 250+20, 15, 0, "#00aaff"))
        # Car 2 (Other) moving Left (other lane)
        self.vehicles.append(Vehicle(2, 900, 250-20, 15, math.pi, "#ff4444"))

    def draw_vehicle(self, v):
        # Draw car body
        l = 30
        w = 15
        x, y = v.x, v.y
        
        # Simple rotation logic for drawing
        cos_h = math.cos(v.heading)
        sin_h = math.sin(v.heading)
        
        p1 = (x + l*cos_h - w*sin_h, y + l*sin_h + w*cos_h)
        p2 = (x - l*cos_h - w*sin_h, y - l*sin_h + w*cos_h)
        p3 = (x - l*cos_h + w*sin_h, y - l*sin_h - w*cos_h)
        p4 = (x + l*cos_h + w*sin_h, y + l*sin_h - w*cos_h)
        
        self.canvas.create_polygon(p1, p2, p3, p4, fill=v.color, outline="white")
        self.canvas.create_text(x, y, text=f"ID:{v.id}", fill="white", font=("Arial", 8, "bold"))
        
        # Draw "Radio Waves"
        if int(time.time() * 10) % 5 == 0:
            self.canvas.create_oval(x-60, y-60, x+60, y+60, outline=v.color, width=1)

    def animate(self):
        if not self.running: return

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        self.canvas.delete("cars") # Clear previous frame cars
        
        # 1. Physics Update
        for v in self.vehicles:
            v.update(dt)
        
        # 2. V2V Communication & Logic (The Core Software)
        ego = self.vehicles[0] if len(self.vehicles) > 0 else None
        
        min_ttc = float('inf')
        threat_id = -1
        
        if ego:
            ego_bsm = ego.get_bsm()
            
            for other in self.vehicles:
                if other.id == ego.id: continue
                
                # Receive BSM
                other_bsm = other.get_bsm()
                
                # Run Collision Algorithm
                ttc = calculate_ttc(
                    [ego_bsm['x'], ego_bsm['y'], ego_bsm['vx'], ego_bsm['vy']],
                    [other_bsm['x'], other_bsm['y'], other_bsm['vx'], other_bsm['vy']]
                )
                
                if ttc < min_ttc:
                    min_ttc = ttc
                    threat_id = other.id

        # 3. UI Update based on Logic
        self.canvas.delete("all")
        # Redraw roads (lazy way, better to use tags but this is simple)
        w = 1000; h = 500
        self.canvas.create_rectangle(0, h/2-40, w, h/2+40, fill="#333333", outline="")
        self.canvas.create_line(0, h/2, w, h/2, fill="white", dash=(20, 20))
        self.canvas.create_rectangle(w/2-40, 0, w/2+40, h, fill="#333333", outline="")
        self.canvas.create_line(w/2, 0, w/2, h, fill="white", dash=(20, 20))

        for v in self.vehicles:
            self.draw_vehicle(v)
            
        if min_ttc < 1.5:
            self.lbl_status.config(text=f"⚠️ COLLISION WARNING (TTC: {min_ttc:.1f}s)", fg="red")
            self.configure(bg="#330000") # Flash background
            # Auto-brake simulation
            if ego: ego.braking = True
        elif min_ttc < 3.0:
            self.lbl_status.config(text=f"CAUTION (TTC: {min_ttc:.1f}s)", fg="yellow")
            self.configure(bg="#1e1e1e")
        else:
            self.lbl_status.config(text="SYSTEM ACTIVE - SCANNING", fg="#00ff00")
            self.configure(bg="#1e1e1e")
            
        if ego:
            self.lbl_info.config(text=f"Ego Speed: {ego.speed*3.6:.1f} km/h | Pos: ({ego.x:.0f}, {ego.y:.0f})")

        self.after(30, self.animate)

if __name__ == "__main__":
    app = V2VSimulator()
    app.mainloop()
