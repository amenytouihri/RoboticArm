"""
Parallel Robot Controller + GUI (FK-calibrated + Jacobian IK)
Initially we had a simple IK based on geometric formulas. However, this
approach can lead to abrupt joint angle changes and instability. So we aborted the analytical IK
and implemented a Jacobian-based differential IK method, which provides smooth and stable motion. 

Upgrades:

- IK uses Jacobian-based differential method for smooth paths
- FK uses per-motor SIGN and OFFSET to match real hardware
- FK picks correct circle intersection branch using previous XY (prevents flips)
- GUI includes a simple "Calibration" panel to test FK against measured poses
"""

import serial
import time
from tkinter import *
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
import re
from serial.tools import list_ports
import numpy as np

# -------------------- 1. Robot Measurements --------------------
L0 = 23.144      # HALF distance between motor centers [mm]
L1 = 61.0        # Link 1 length [mm]
L2 = 93.6        # Link 2 length [mm]
L4 = 50.91799985 # working plane offset relative to motor axis [mm]
# Convention used below: working_y = motor_frame_y - L4

HOME_ANGLE_A = 90.0
HOME_ANGLE_B = 90.0

# -------------------- 2. FK Calibration Knobs  --------------------
# These two lines fix 90° direction mismatch:
# - OFFSET rotates the motor-frame angle to match the real "0 direction"
# - SIGN flips direction if the increasing angle goes opposite of math convention

# Sign and Offset calibration:
SIGN_A = +1  
SIGN_B = -1   

OFFSET_A_DEG = 0.0   
OFFSET_B_DEG = 0.0 
# -------------------- 2b. Joint limits & angle safety --------------------

# Adjust these to match the physical robot
JOINT_A_MIN = 30.0
JOINT_A_MAX = 150.0

JOINT_B_MIN = 30.0
JOINT_B_MAX = 150.0

def wrap_angle_deg(a):
    
    # Wrap angle to [-180, 180] degrees to prevent full rotations. 
   
    return ((a + 180) % 360) - 180

# Branch selection:
# Choosing the intersection closest to the previous XY, which prevents flipping.

'''
# -------------------- 2. Kinematics Functions --------------------

def _ik_two_link(x_local, y_local, l1, l2, elbow_sign=+1):
# Inverse kinematics for a 2-link planar arm.
# elbow_sign: +1 for elbow-up, -1 for elbow-down configuration
    d_sq = x_local**2 + y_local**2
    d = math.sqrt(d_sq)
    # Check reachability
    if d > (l1 + l2) or d < abs(l1 - l2):
        raise ValueError("Unreachable for this side")

    cos_beta = (d_sq - l1**2 - l2**2) / (2 * l1 * l2)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    sin_beta = elbow_sign * math.sqrt(max(0.0, 1.0 - cos_beta**2))
    beta = math.atan2(sin_beta, cos_beta)
    # Angle from base to target point
    alpha = math.atan2(y_local, x_local)
    # Compute motor angle
    theta = alpha - math.atan2(l2 * sin_beta, l1 + l2 * cos_beta)
    return theta


def ik(x, y):
    # Convert working-plane coordinates to motor frame
    y_m = y + L4
    # Left motor
    x_left  = x + L0
    y_left  = y_m
    x_right = x - L0
    y_right = y_m
    try:
    #§ Compute IK for both sides
        theta1 = _ik_two_link(x_left,  y_left,  L1, L2, elbow_sign=+1)
        theta2 = _ik_two_link(x_right, y_right, L1, L2, elbow_sign=+1)
        return math.degrees(theta1), math.degrees(theta2)
    except ValueError:
        return None, None
'''


# -------------------- 3. Forward Kinematics (calibrated + stable branch) --------------------
def fk(theta1_deg, theta2_deg, prev_xy=None):
    """
    Forward kinematics for parallel robot.

    Inputs:
    - theta1_deg, theta2_deg : motor angles in degrees
    - prev_xy                : previous end-effector position

    Outputs:
    - (x, y) in working-plane coordinates (mm)

    Features:
    - hardware calibration via SIGN and OFFSET
    - stable branch selection using previous position
    """
    # Apply sign + offset to match physical direction and 0-axis
    # Convert commanded motor angles into calibrated motor-frame angles
    # that match the real physical orientation of the robot.

    t1 = SIGN_A * theta1_deg + OFFSET_A_DEG
    t2 = SIGN_B * theta2_deg + OFFSET_B_DEG

    theta1 = math.radians(t1)
    theta2 = math.radians(t2)

    # Motor anchors
    A1x, A1y = -L0, 0.0
    A2x, A2y =  L0, 0.0

    # Elbows after proximal links
    P1x = A1x + L1 * math.cos(theta1)
    P1y = A1y + L1 * math.sin(theta1)

    P2x = A2x + L1 * math.cos(theta2)
    P2y = A2y + L1 * math.sin(theta2)

    # Distance between elbows
    dx = P2x - P1x
    dy = P2y - P1y
    d = math.hypot(dx, dy)

    # Distal links must be able to meet
    if d > 2 * L2 or d < 1e-9:
        return None, None

    # Circle-circle intersection (equal radii L2)
    a = d / 2.0
    h_sq = L2**2 - a**2
    if h_sq < 0:
        return None, None
    h = math.sqrt(h_sq)

    # Midpoint between elbows
    Mx = (P1x + P2x) / 2.0
    My = (P1y + P2y) / 2.0

    # Unit perpendicular to (P2 - P1)
    ux = -dy / d
    uy =  dx / d

    # Two possible intersections in motor frame
    Ex1, Ey1 = Mx + h * ux, My + h * uy
    Ex2, Ey2 = Mx - h * ux, My - h * uy

    # --- ELBOW CONFIGURATION LOCK ---
    # Branch locking:
    # The robot has two valid FK solutions.
    # Without branch selection, FK may "flip" between them.
    # We enforce consistency by choosing the solution that
    # stays on the same side of the elbow line.

    # Keep end-effector on the SAME side of the elbow line
    # Choose the solution with Ey ABOVE the elbow midpoint My
    if Ey1 >= My:
        Ex, Ey = Ex1, Ey1
    else:
        Ex, Ey = Ex2, Ey2

    

    # Convert to working plane (Y axis flipped + offset)
    X = Ex
    Y = -(Ey - L4)
    # Convert from motor frame to working plane:
    # - flip Y axis
    # - apply vertical offset L4


    return X, Y


# -------------------- 4. Jacobian + Differential IK --------------------
def jacobian_numeric_parallel(theta1_deg, theta2_deg, prev_xy=None, eps_deg=0.1):
    """
    Numerical Jacobian using fk().
    Returns J in units of (mm/deg), or None if FK invalid around the point.

    #Numerical Jacobian using existing fk()
    eps_deg is a tiny angle change (default 0.1°) used for finite differences. it affects Jacobian accuracy, not robot motion directly.
    Returns J in units of (mm/deg).

    """
    x0, y0 = fk(theta1_deg, theta2_deg, prev_xy=prev_xy)
    if x0 is None:
        return None
    #Calls for FK. If FK fails, it returns None

    # Partial derivatives via central differences d/dtheta1
    x_p, y_p = fk(theta1_deg + eps_deg, theta2_deg, prev_xy=(x0, y0))
    x_m, y_m = fk(theta1_deg - eps_deg, theta2_deg, prev_xy=(x0, y0))
    # Check validity of FK at perturbed points 
    if x_p is None or x_m is None:
        return None
    # Partial w.r.t. theta1
    #We slightly nudge θ1 only, holding θ2 constant.+eps_deg gives forward perturbation & -eps_deg gives backward perturbation
    #Why do both + and -? Because it’s more accurate than only forward difference
    dx_d1 = (x_p - x_m) / (2 * eps_deg)
    dy_d1 = (y_p - y_m) / (2 * eps_deg)

    '''
    Central difference equation - The central difference formula estimates a derivative by comparing the output
    when we slightly increase and slightly decrease an input,and we use it here to approximate the Jacobian because our
    robot’s forward kinematics has no simple analytic derivative.
    '''
    # Partial w.r.t. theta2 
    x_p, y_p = fk(theta1_deg, theta2_deg + eps_deg, prev_xy=(x0, y0))
    x_m, y_m = fk(theta1_deg, theta2_deg - eps_deg, prev_xy=(x0, y0))
    # Check validity of FK at perturbed points 
    if x_p is None or x_m is None:
        return None
    #Nudge θ2 only, holding θ1 constant

    dx_d2 = (x_p - x_m) / (2 * eps_deg)
    dy_d2 = (y_p - y_m) / (2 * eps_deg)
    # Construct Jacobian matrix
    return np.array([[dx_d1, dx_d2],
                     [dy_d1, dy_d2]], dtype=float)

def ik_differential_parallel_step(theta1_deg, theta2_deg, x_target, y_target,
                                  prev_xy=None,
                                  alpha=0.6, eps_deg=0.1, damping=1e-4):
    #Alpha is step size. We use damped least squares for stability near singularities.

    """
    One Jacobian-based IK step (damped least squares).
    Returns (new_theta1_deg, new_theta2_deg) or (None, None).
    
    Perform one damped least-squares IK step.

    Strategy:
    - compute FK error in Cartesian space
    - map error to joint space via Jacobian
    - apply small incremental correction (alpha)
    - enforce joint limits and angle wrapping
    """
    # Current end-effector position
    # Use previous xy to keep branch consistent 
    
    xe, ye = fk(theta1_deg, theta2_deg, prev_xy=prev_xy)
    if xe is None:
        return None, None
    # Compute position error 
    delta_x = np.array([x_target - xe, y_target - ye], dtype=float)
# Compute Jacobian
# Numerical Jacobian around current angles
    J = jacobian_numeric_parallel(theta1_deg, theta2_deg, prev_xy=(xe, ye), eps_deg=eps_deg)
    if J is None:
        return None, None
    '''
    delta_theta = J⁻¹ × delta_x
    but we avoid using the inverse directly because if we use it directly, crash control can happen
    SO this way we avoid huge angle jumps,...etc
    Damped least squares: delta_theta = (J^T J + λI)^-1 J^T delta_x
    '''
    JTJ = J.T @ J
    rhs = J.T @ delta_x
    #J.T is jacobian transpose, just flips rows and columns, @ is multiplication symbol for matrices. 
    # λI adds safety so nothing explodes
    try:
        delta_theta = np.linalg.solve(JTJ + damping * np.eye(2), rhs)
    except np.linalg.LinAlgError:
        return None, None
    
    # Apply incremental update with step size alpha 
    new_t1 = theta1_deg + alpha * float(delta_theta[0])
    new_t2 = theta2_deg + alpha * float(delta_theta[1])

    # --- HARD JOINT LIMITS (prevent insane poses) ---
    new_t1 = max(JOINT_A_MIN, min(JOINT_A_MAX, new_t1))
    new_t2 = max(JOINT_B_MIN, min(JOINT_B_MAX, new_t2))

    # --- PREVENT FULL ROTATIONS ---
    new_t1 = wrap_angle_deg(new_t1)
    new_t2 = wrap_angle_deg(new_t2)

    return new_t1, new_t2

# -------------------- 5. Arduino Communication Class --------------------
class ArduinoCommunication:
    """
    Handles communication with Arduino.

    Features:
    - automatic port detection
    - graceful fallback to simulation mode
    - unified send/read interface
    """
    def __init__(self, port=None, baudrate=115200, timeout=0.1):
        self._simulated_data = []
        # Auto-detect serial port 
        available_ports = [
            p.device for p in list_ports.comports()
            if ('usbmodem' in p.device) or ('usbserial' in p.device) or ('COM' in p.device)
        ]
        # Use specified port or first available 
        if port is None:
            port = available_ports[0] if available_ports else None

        self.arduino = None
        if port:
            try:
                # Establish serial connection 
                self.arduino = serial.Serial(port, baudrate, timeout=timeout)
                # Allow time for Arduino to reset
                time.sleep(2)
                # Clear input buffer
                self.arduino.reset_input_buffer()
                
                print(f"Connected to Arduino on port {port}")
            except serial.SerialException:
                self.arduino = None
                print("Serial connection failed. Running in simulation mode.")
        else:
            print("No serial port found. Running in simulation mode.")
    # Send data to Arduino or simulate it 
    def send_data(self, data):
        
        if self.arduino and self.arduino.is_open:
            try:
                # Send data as UTF-8 encoded line 
                self.arduino.write((str(data) + "\n").encode("utf-8"))
                self.arduino.flush()
            except Exception:
                pass
        else:
            # Simulation mode: emulate Arduino debug format
            try:
                # Parse input angles and create fake feedback line as Arduino would send back  
                theta1, theta2 = [float(i) for i in str(data).split(',')]
                # Simulated feedback format
                self._simulated_data.append(
                    f"A: {theta1:.2f} | PWM: 120 | VTarget: {theta1:.2f} | Error: 0.00 "
                    f"|| B: {theta2:.2f} | PWM: 120 | VTarget: {theta2:.2f} | Error: 0.00"
                )
            except Exception:
                pass
    # Read data from Arduino or simulated buffer
    # Returns a line of text or None 
    def read_data(self):
        if self.arduino and self.arduino.in_waiting > 0:
    # Read line from Arduino
            try:
                return self.arduino.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                return None
            # Simulation mode: return next simulated line if available
        elif (not self.arduino) and self._simulated_data:
            return self._simulated_data.pop(0)
        return None

    def close(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()

# -------------------- 6. GUI --------------------
class GUI:
    """
    Tkinter-based GUI for:
    - sending Cartesian waypoints
    - visualizing target vs actual path
    - FK calibration testing
    - real-time Arduino feedback
    """
    # Initialize GUI components
    def __init__(self, microcontroller: ArduinoCommunication):
        self.microcontroller = microcontroller
        # Tkinter setup
        # Main window
        self.root = Tk()
        self.root.title("Five-bar Parallel Robot Controller (FK-Calibrated + Jacobian IK)")
        self.root.geometry("740x880")
        # Layout 
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(N, W, E, S))
        # Input row
        ttk.Label(self.frame, text="Enter coordinates (X,Y;X2,Y2;...):").grid(row=0, column=0, sticky=W)
        self.input_box = ttk.Entry(self.frame, width=36)
        self.input_box.grid(row=1, column=0, sticky=(W, E), padx=5, pady=5)
        self.input_box.bind("<Return>", lambda e: self.send_to_arduino())
        # Buttons
        ttk.Button(self.frame, text="Send Coords (IK)", command=self.send_to_arduino).grid(row=1, column=1, padx=5)
        ttk.Button(self.frame, text="Home (90,90)", command=self.go_home).grid(row=1, column=2, padx=5)
        # Feedback label
        self.feedback_label = ttk.Label(self.frame, text="Ready.")
        self.feedback_label.grid(row=2, column=0, columnspan=3, sticky=W, pady=5)

        # Calibration quick test row
        ttk.Label(self.frame, text="Calibration angles A,B:").grid(row=3, column=0, sticky=W)
        self.cal_entry = ttk.Entry(self.frame, width=18)
        self.cal_entry.insert(0, "90,90")
        self.cal_entry.grid(row=3, column=1, sticky=W)
        ttk.Button(self.frame, text="Send Angles (no IK)", command=self.send_angles_only).grid(row=3, column=2, padx=5)
        # FK output label
        self.cal_out = ttk.Label(self.frame, text="FK(A,B) = ?")
        self.cal_out.grid(row=4, column=0, columnspan=3, sticky=W)
        # Arduino log text box
        ttk.Label(self.frame, text="Arduino Serial Log:").grid(row=5, column=0, columnspan=3, sticky=W)
        self.received_text = Text(self.frame, height=8, width=86)
        self.received_text.grid(row=6, column=0, columnspan=3, padx=5, pady=5)

        # Plot
        self.fig = Figure(figsize=(6.6, 4.6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Target Path (Blue) vs Actual FK from Arduino Angles (Red)")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.line_target, = self.ax.plot([], [], 'bo-', markersize=3, label="Target Path")
        self.line_actual, = self.ax.plot([], [], 'rx', markersize=6, label="Actual Position")
        self.ax.legend()
        # Set plot limits based on robot reach
        self.max_reach = L0 + L1 + L2
        self.ax.set_xlim(-self.max_reach, self.max_reach)
        self.ax.set_ylim(-self.max_reach * 0.6, self.max_reach)  # allow negative
        self.ax.axhline(0, color='gray', linestyle='--', linewidth=0.5)
        # Vertical offset line
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

        # State
        self.x_target, self.y_target = [], []
        self.x_actual, self.y_actual = [], []
        # Current joint angles
        self.theta1 = HOME_ANGLE_A
        self.theta2 = HOME_ANGLE_B

        # Initialize current xy from FK(home)
        hx, hy = fk(self.theta1, self.theta2, prev_xy=None)
        # Initialize current position
        self.current_x = hx if hx is not None else 0.0
        self.current_y = hy if hy is not None else 0.0
        # Max waypoints per command
        self.MAX_WAYPOINTS = 50
        self.poll_arduino()

    def send_angles_only(self):
        """
        Directly send A,B angles (no IK). Useful for FK calibration.
        # Allows bypassing IK to directly test FK calibration
        # against real hardware measurements.

        """
        # Parse input
        txt = self.cal_entry.get().strip()
        # Regex to match "A,B" format 
        # Allows optional spaces and decimal points
        m = re.match(r'^\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*$', txt)
        if not m:
            self.feedback_label.config(text="Calibration format invalid. Use: A,B")
            return
        a = float(m.group(1))
        b = float(m.group(2))

    # Clamp manual commands too (safety)
        a = max(JOINT_A_MIN, min(JOINT_A_MAX, a))
        b = max(JOINT_B_MIN, min(JOINT_B_MAX, b))
        
        cmd = f"{a:.2f},{b:.2f}"
        self.microcontroller.send_data(cmd)
        self.feedback_label.config(text=f"Sent angles: {cmd} (no IK)")

        # Show FK prediction immediately (using last known xy for stable branch)
        px, py = self.current_x, self.current_y
        # Compute FK
        # Use previous xy to keep branch consistent
        # Show FK output
        x_fk, y_fk = fk(a, b, prev_xy=(px, py))
        # Display FK result
        # Handle invalid FK case
        if x_fk is None:
            self.cal_out.config(text=f"FK({a:.2f},{b:.2f}) = invalid (geometry/branch)")
        else:
            self.cal_out.config(text=f"FK({a:.2f},{b:.2f}) = ({x_fk:.2f}, {y_fk:.2f}) mm")

    def send_to_arduino(self):
        # Parse input coordinates
        # Expected format: "X1,Y1;X2,Y2;..."
        user_input = self.input_box.get().strip()
        if not user_input:
            self.feedback_label.config(text="Please enter coordinates!")
            return
        # Regex to extract coordinate pairs
        # Allows optional spaces and decimal points
        coord_pairs = re.findall(r'(-?\d+\.?\d*),(-?\d+\.?\d*)', user_input)
        if not coord_pairs:
            self.feedback_label.config(text="Invalid format. Use: X,Y;X2,Y2;...")
            return
        # Clear previous target path
        self.x_target.clear()
        self.y_target.clear()
        # Start from current position
        start_x = self.current_x
        start_y = self.current_y
        # Multiple small IK steps per waypoint:
        # improves convergence and avoids large joint jumps.

        IK_ITERS_PER_WAYPOINT = 6
        # IK parameters
        alpha   = 0.55
        damping = 1e-4
        eps_deg = 0.1
        # Process each coordinate pair
        for x_str, y_str in coord_pairs:
            end_x = float(x_str)
            end_y = float(y_str)
        # Interpolate waypoints between start and end
            x_path = np.linspace(start_x, end_x, self.MAX_WAYPOINTS)
            y_path = np.linspace(start_y, end_y, self.MAX_WAYPOINTS)
        # Perform IK for each waypoint
            for x_i, y_i in zip(x_path, y_path):
                ok = True
                # Multiple small IK steps per waypoint
                for _ in range(IK_ITERS_PER_WAYPOINT):
                    # Use previous xy to keep branch consistent
                    prev_xy = (self.current_x, self.current_y)
                    # One IK step
                    t1, t2 = ik_differential_parallel_step(
                        self.theta1, self.theta2,
                        x_i, y_i,
                        prev_xy=prev_xy,
                        alpha=alpha,
                        eps_deg=eps_deg,
                        damping=damping
                    )
                    # Check for IK failure
                    if t1 is None:
                        ok = False
                        break
                    self.theta1, self.theta2 = t1, t2

                    # update internal xy estimate to keep branch stable
                    xf, yf = fk(self.theta1, self.theta2, prev_xy=prev_xy)
                    if xf is not None:
                        self.current_x, self.current_y = xf, yf

                if not ok:
                    # IK failed for this waypoint
                    # Notify user and abort
                    # Show failure message
                    self.feedback_label.config(text=f"IK failed near ({x_i:.2f}, {y_i:.2f}). Try closer points.")
                    return
                # Send angles to Arduino 
                # Format: "theta1,theta2"
                angles_to_send = f"{self.theta1:.2f},{self.theta2:.2f}"
                self.microcontroller.send_data(angles_to_send)
                # Append to target path for plotting
                self.x_target.append(x_i)
                self.y_target.append(y_i)
                time.sleep(0.015)
            # Update start point for next segment
            start_x, start_y = end_x, end_y
            self.feedback_label.config(text=f"Segment completed to ({end_x:.2f}, {end_y:.2f}).")
        # Clear input box after sending
        self.input_box.delete(0, END)
        self.line_target.set_data(self.x_target, self.y_target)
        self.canvas.draw_idle()
    # Go to home position (90,90)
    def go_home(self):
    # Send home command to Arduino
        cmd = f"{HOME_ANGLE_A:.2f},{HOME_ANGLE_B:.2f}"
        # Update internal state
        self.microcontroller.send_data(cmd)
        # Reset internal angles
        self.theta1, self.theta2 = HOME_ANGLE_A, HOME_ANGLE_B
        hx, hy = fk(self.theta1, self.theta2, prev_xy=None)
        # Update current position
        if hx is not None:
            self.current_x, self.current_y = hx, hy
        # Update target path
        self.x_target[:] = [self.current_x]
        self.y_target[:] = [self.current_y]
        self.line_target.set_data(self.x_target, self.y_target)
        self.canvas.draw_idle()
        self.feedback_label.config(text=f"Sent Home: {cmd}")

    def poll_arduino(self):
        # Periodically poll Arduino for feedback and update actual path plot
        # The red crosses show the REAL end-effector position,
        # reconstructed from measured motor angles via FK.

        try:
        # Read line from Arduino
            data = self.microcontroller.read_data()
            # Parse feedback line for angles
            if data and isinstance(data, str):
                # Append to log
                self.received_text.insert(END, f"{data}\n")
                self.received_text.see(END)
                # Regex to extract "A: angle || B: angle"
                match = re.search(r'A:\s*(-?\d+\.?\d*).*?\|\|\s*B:\s*(-?\d+\.?\d*)', data)
                if match:
                    theta1_actual = float(match.group(1))
                    theta2_actual = float(match.group(2))

                    # Sync internal angles to measured
                    self.theta1 = theta1_actual
                    self.theta2 = theta2_actual

                    # Use previous point to keep branch consistent
                    prev_xy = (self.current_x, self.current_y)
                    x_actual, y_actual = fk(theta1_actual, theta2_actual, prev_xy=prev_xy)
                    # Append to actual path for plotting
                    if x_actual is not None:
                        self.current_x, self.current_y = x_actual, y_actual
                        self.x_actual.append(x_actual)
                        self.y_actual.append(y_actual)
                        # Limit number of points to avoid clutter
                        MAX_POINTS = 60
                        if len(self.x_actual) > MAX_POINTS:
                            self.x_actual = self.x_actual[-MAX_POINTS:]
                            self.y_actual = self.y_actual[-MAX_POINTS:]
                        # Update actual path plot
                        self.line_actual.set_data(self.x_actual, self.y_actual)
                        self.canvas.draw_idle()

        except Exception:
            pass

        self.root.after(50, self.poll_arduino)

    def run(self):
        self.root.mainloop()

# -------------------- Main --------------------
if __name__ == "__main__":
    # Print calibration constants at startup
    # to ensure software and hardware configuration are in sync.

    print("Calibration constants:")
    print("SIGN_A, SIGN_B =", SIGN_A, SIGN_B)
    print("OFFSET_A_DEG, OFFSET_B_DEG =", OFFSET_A_DEG, OFFSET_B_DEG)
    # Initialize communication and GUI
    micro = ArduinoCommunication()
    gui = GUI(micro)
    # Initialize to home position
    # Send home once
    micro.send_data(f"{HOME_ANGLE_A:.2f},{HOME_ANGLE_B:.2f}")
    time.sleep(0.05)
    # Run GUI
    try:
        gui.run()
    finally:
        micro.close()
