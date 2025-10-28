import tkinter as tk
from tkinter import ttk, messagebox
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from cflib.crazyflie import Crazyflie
import cflib.crtp
import time
import threading

#CRTP connection 
cflib.crtp.init_drivers()
cf = Crazyflie(rw_cache='./cache')

def connected(link_uri):
    print("Connected to", link_uri)

uri = 'udp://192.168.43.42:2390' 
cf.open_link(uri)

needToStop = 0

#Flight control commands
def send_hover(thrust=38000, roll=0, pitch=0, yawrate=0, duration=0.1):
    """Send a single hover setpoint for given duration"""
    global needToStop
    start = time.time()
    while time.time() - start < duration:
        if needToStop:
            cf.commander.send_stop_setpoint()
            return
        cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        time.sleep(0.05)

def turnDrone(degrees, yawrate=60):
    """Turn by sending yaw setpoints"""
    print(f"Turning drone by {degrees:.1f}°")
    duration = abs(degrees) / yawrate  # seconds = degrees / deg/s
    direction = 1 if degrees > 0 else -1
    send_hover(38000, 0, 0, yawrate * direction, duration)
    send_hover(38000, 0, 0, 0, 0.2)

def goForward(metres, pitch_angle=5):
    """Move forward by pitching forward"""
    print(f"Flying forward {metres:.2f} m")
    speed = 0.5  # approx m/s depending on tuning
    duration = abs(metres / speed)
    direction = 1 if metres > 0 else -1
    send_hover(38000, 0, pitch_angle * direction, 0, duration)
    send_hover(38000, 0, 0, 0, 0.2)

def move_vertical(delta_m):
    """Move up or down by adjusting thrust briefly"""
    print(f"Changing altitude by {delta_m:.2f} m")
    hover_thrust = 38000
    thrust_change = 3000 * delta_m
    thrust = hover_thrust + thrust_change
    send_hover(thrust, 0, 0, 0, 0.5)
    send_hover(hover_thrust, 0, 0, 0, 0.2)

def move_direction(direction):
    """Manual arrow control"""
    if direction == "up":
        move_vertical(+0.3)
    elif direction == "down":
        move_vertical(-0.3)
    elif direction == "forward":
        goForward(0.5)
    elif direction == "back":
        goForward(-0.5)
    elif direction == "left":
        turnDrone(-15)
    elif direction == "right":
        turnDrone(+15)

#GUI definition
class DronePathPlanner:
    global needToStop
    def __init__(self, root):
        self.root = root
        self.root.title("Surveillance Drone: Flight Planner (CRTP version)")
        self.points = []
        self.steps = []
        self.create_layout()
        self.setup_plot()

    def create_layout(self):
        self.frame_plot = ttk.Frame(self.root)
        self.frame_plot.grid(row=0, column=0, sticky="nsew")
        self.frame_sidebar = ttk.Frame(self.root, width=250)
        self.frame_sidebar.grid(row=0, column=1, sticky="ns")
        self.frame_buttons = ttk.Frame(self.root)
        self.frame_buttons.grid(row=1, column=0, columnspan=2, sticky="ew")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        ttk.Label(self.frame_sidebar, text="Flight Steps", font=("Arial", 12, "bold")).pack(pady=5)
        self.tree = ttk.Treeview(self.frame_sidebar, columns=("angle", "dist", "alt"), show="headings", height=15)
        self.tree.heading("angle", text="Turn (°)")
        self.tree.heading("dist", text="Distance (m)")
        self.tree.heading("alt", text="Altitude (m)")
        self.tree.pack(fill="both", expand=True, padx=5)
        ttk.Button(self.frame_sidebar, text="Add Altitude Step", command=self.add_altitude_step).pack(pady=5)
        ttk.Button(self.frame_sidebar, text="Start Mission", command=self.start_mission).pack(pady=10)

        buttons = [
            ("Take Off", self.takeoff),
            ("Land", self.land),
            ("Kill Props", self.kill),
            ("Up", lambda: move_direction("up")),
            ("Down", lambda: move_direction("down")),
            ("Forward", lambda: move_direction("forward")),
            ("Back", lambda: move_direction("back")),
            ("Left", lambda: move_direction("left")),
            ("Right", lambda: move_direction("right")),
        ]
        for (label, cmd) in buttons:
            ttk.Button(self.frame_buttons, text=label, command=cmd, width=10).pack(side="left", padx=3, pady=3)

    def setup_plot(self):
        fig = Figure(figsize=(5, 5))
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Draw Flight Path (click to add points)")
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.ax.grid(True)
        self.canvas = FigureCanvasTkAgg(fig, master=self.frame_plot)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_click)

        # Altitude entry box
        self.altitude_frame = ttk.Frame(self.frame_plot)
        self.altitude_frame.pack(pady=5)
        ttk.Label(self.altitude_frame, text="Desired Altitude (m):").pack(side=tk.LEFT)
        self.altitude_entry = ttk.Entry(self.altitude_frame, width=8)
        self.altitude_entry.pack(side=tk.LEFT, padx=5)
        ttk.Button(self.altitude_frame, text="Set Altitude", command=self.set_altitude).pack(side=tk.LEFT)

    def on_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        x, y = event.xdata, event.ydata
        if self.points:
            x1, y1 = self.points[-1]
            dx, dy = x - x1, y - y1
            distance = math.sqrt(dx**2 + dy**2)
            angle = math.degrees(math.atan2(dy, dx))
            if len(self.points) >= 2:
                prev_dx = self.points[-1][0] - self.points[-2][0]
                prev_dy = self.points[-1][1] - self.points[-2][1]
                prev_angle = math.degrees(math.atan2(prev_dy, prev_dx))
                turn_angle = angle - prev_angle
            else:
                turn_angle = angle
            self.steps.append({"turn": turn_angle, "dist": distance, "alt": 0})
            self.tree.insert("", "end", values=(f"{turn_angle:.1f}", f"{distance:.2f}", "0"))
            self.ax.plot([x1, x], [y1, y], 'b-o')
            self.canvas.draw()
        self.points.append((x, y))

    def add_altitude_step(self):
        if not self.steps:
            messagebox.showinfo("Info", "Add at least one XY step first.")
            return
        self.steps.append({"turn": 0, "dist": 0, "alt": 1})
        self.tree.insert("", "end", values=("0", "0", "1"))

    def set_altitude(self):
        try:
            alt = float(self.altitude_entry.get())
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter a number for altitude")
            return
        if not self.steps:
            self.steps.append({"turn": 0, "dist": 0, "alt": alt})
            self.tree.insert("", "end", values=("0", "0", f"{alt:.2f}"))
        else:
            self.steps[-1]["alt"] = alt
            last_item = self.tree.get_children()[-1]
            self.tree.item(last_item, values=(
                f"{self.steps[-1]['turn']:.1f}",
                f"{self.steps[-1]['dist']:.2f}",
                f"{self.steps[-1]['alt']:.2f}"
            ))
        print(f"Next step altitude set to {alt} m")

    def takeoff(self):
        global needToStop
        needToStop = 0
        threading.Thread(target=gentle_takeoff, args=(cf,), daemon=True).start()

    def land(self):
        print("Landing...")
        for thrust in range(38000, 20000, -500):
            if needToStop:
                cf.commander.send_stop_setpoint()
                break
            cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(0.1)
        cf.commander.send_stop_setpoint()
        print("Landed")

    def kill(self):
        print("Killing propellers!")
        global needToStop
        needToStop = 1
        cf.commander.send_stop_setpoint()

    def start_mission(self):
        TURN_SPEED_DEG_PER_S = 90.0
        MOVE_SPEED_M_PER_S = 0.5
        VERT_SPEED_M_PER_S = 0.5 

        if not self.steps:
            messagebox.showinfo("Info", "No steps defined.")
            return
        print("\n--- Starting Autonomous Flight ---")
        for step in self.steps:
            # Turn
            if step["turn"] != 0:
                turnDrone(step["turn"])
                wait_time = abs(step["turn"]) / TURN_SPEED_DEG_PER_S
                time.sleep(wait_time)

            # Move forward
            if step["dist"] != 0:
                goForward(step["dist"])
                wait_time = abs(step["dist"]) / MOVE_SPEED_M_PER_S
                time.sleep(wait_time)

            # Altitude change
            if step["alt"] != 0:
                move_vertical(step["alt"])
                wait_time = abs(step["alt"]) / VERT_SPEED_M_PER_S
                time.sleep(wait_time)
        cf.commander.send_stop_setpoint()
        print("--- Mission complete ---")

def gentle_takeoff(cf):
    print("Takeoff...")
    for thrust in range(20000, 38000, 1000):
        if needToStop:
            cf.commander.send_stop_setpoint()
            return
        cf.commander.send_setpoint(0, 0, 0, thrust)
        time.sleep(0.05)
    print("Hovering...")
    send_hover(38000, 0, 0, 0, 2)
    print("Takeoff complete.")

if __name__ == "__main__":
    root = tk.Tk()
    app = DronePathPlanner(root)
    root.mainloop()
