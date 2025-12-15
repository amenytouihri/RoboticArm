# this entire code works to control the 2-link parallel robot through input of a series of coordinates
# form x,y;x1,y1 etc

# To install necessary libraries:
#pip install pyserial
#pip install scipy

#setup of all packages and functions required for the code to run
import serial
import time
from tkinter import *
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import random

import math
import re  # For parsing the multiple coordinate inputs
from scipy.optimize import fsolve # Corrected module name



# robot measurements
L0 = 20  # base offset (l0)
L1 = 60  # link 1 length (l1)
L2 = 60  # link 2 length (l2)

def calculate_ik(x_target, y_target):

    # calculation of theta1 for left arm - see w5 PP
    x_vector_1 = x_target + L0
    y_vector_1 = y_target
    D1_sq = x_vector_1 ** 2 + y_vector_1 ** 2
    D1 = math.sqrt(D1_sq)

    # check for reachability
    if D1 > (L1 + L2) or D1 < abs(L1 - L2):
        print(f"Target ({x_target},{y_target}) is unreachable by left leg.")
        return None, None #stops calculations from continuing if the target point isn't reachable

    # calculation of angles, see inverse kinematics matlab
    phi1 = math.atan2(y_vector_1, x_vector_1)
    alpha1 = math.acos((L1**2 + D1_sq - L2**2) / (2 * L1 * D1))
    theta_a1_rad = phi1 - alpha1

    # repeat for right leg
    x_vector_2 = x_target - L0
    y_vector_2 = y_target
    D2_sq = x_vector_2 ** 2 + y_vector_2 ** 2
    D2 = math.sqrt(D2_sq)

    # check for reachability
    if D2 > (L1 + L2) or D2 < abs(L1 - L2):
        print(f"Target ({x_target},{y_target}) is unreachable by right leg.")
        return None, None

    # calculation of angles, see inverse kinematics matlab
    phi2 = math.atan2(y_vector_2, x_vector_2)
    alpha2 = math.acos((L1**2 + D2_sq - L2**2) / (2 * L1 * D2))
    theta_a2_rad = phi2 - alpha2

    # use in-built functions to convert radians (from arduino) back to degrees for our understanding
    theta_a1_deg = math.degrees(theta_a1_rad)
    theta_a2_deg = math.degrees(theta_a2_rad)

    # return statement to show both angles of final position
    return theta_a1_deg, theta_a2_deg


def calculate_fk(theta_a1_deg, theta_a2_deg):
    # needed to tell us where robot actually is, but not to move it (just used for visualisation)
    # there is another, more precise way to do this that is faster but more complex
    # this method uses iteration until it finds the point where both equations equal 0

    # Convert angles to radians - earlier is only included in calculate_ik function, not global
    theta_a1_rad = math.radians(theta_a1_deg)
    theta_a2_rad = math.radians(theta_a2_deg)

    # pre-calculate cosines and sines of ACTIVE joint angles, makes code simpler later on
    #
    c_a1 = math.cos(theta_a1_rad)
    s_a1 = math.sin(theta_a1_rad)
    c_a2 = math.cos(theta_a2_rad)
    s_a2 = math.sin(theta_a2_rad)

    # essentially uses the fact that two circles with centres around each l1 end and radii of l2
    # rearrangement of a2 + b2 = c2 to make 0 = a2 + b2 -c2
    # the two circles can then be made equal to each other to find the exact coordinate the end effector is at for the given angles
    def equations(p):
        x, y = p

        # constraint for LEFT leg (Base at -L0, 0)
        # uses the above cos and sin calculations to figure out the position of link 1 end at each angle (this is the two sets of brackets)
        F1 = (x + L0 - L1 * c_a1)**2 + (y - L1 * s_a1)**2 - L2**2

        # constraint for RIGHT leg (Base at L0, 0)
        # uses the above cos and sin calculations to figure out the position of link 1 end at each angle (this is the two sets of brackets)
        F2 = (x - L0 - L1 * c_a2)**2 + (y - L1 * s_a2)**2 - L2**2

        return (F1, F2)

    try:
        # iterative method so start with a good guess (essential for fsolve function speed and efficiency)
        # the midpoint between the ends of the two shoulder links (L1) is our intial guess

        x_guess = (-L0 + L1 * c_a1 + L0 + L1 * c_a2) / 2
        y_guess = (L1 * s_a1 + L1 * s_a2) / 2

        # fsolve function uses the equations defined above and our initial guess to iterate through input values until
        # it finds answers that equal 0 (which for us represents the x and y coordinates for the given angles)
        x_solved, y_solved = fsolve(equations, (x_guess, y_guess))

        # fsolve returns an array, extract the float values
        return float(x_solved), float(y_solved)

    except Exception as e:
        print(f"FK Solver Error: Could not find a solution. {e}")
        # if an error occurs for any reason, it sends a safe position to the GUI that won't break the plot
        return 0.0, 0.0

''' ADD THIS BACK IN FOR ACTUAL ROBOT AND REMOVE ENTIRE BELOW CLASS
# from class scripts, connects code to arduino
class ArduinoCommunication:
    def __init__(self, port="COM4", baudrate=115200, timeout=0.1):
        self.arduino = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # wait for the connection to establish
        self.arduino.reset_input_buffer()

    def send_data(self, data):
        if self.arduino.is_open:
            formatted_data = str(data) + "\n"
            self.arduino.write(formatted_data.encode('utf-8'))
            self.arduino.flush()

    def read_data(self):
        if self.arduino.is_open and self.arduino.in_waiting > 0:
            return self.arduino.readline().decode('utf-8', errors='ignore').strip()
        return None

    def close(self):
        if self.arduino.is_open:
            self.arduino.close()
            '''

# below just allows us to run the GUI without having the arduino connected, will remove this for final submission
class ArduinoCommunication:
    def __init__(self, port="COM4", baudrate=115200, timeout=0.1):
        self._simulated_data = []  # New: Queue for simulated angle feedback

        try:
            # Attempt to open the physical serial port
            self.arduino = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Wait for the connection to establish
            self.arduino.reset_input_buffer()
            print(f"INFO: Successfully connected to Arduino on {port}.")
        except serial.SerialException as e:
            # If connection fails, switch to simulation mode
            print(f"WARNING: Could not open port {port}. Running in simulation mode. ({e})")
            self.arduino = None

    def _simulate_send_receive(self, sent_angle_data):
        """Simulates the Arduino's noisy feedback and adds it to the queue."""
        import random

        # 1. Parse angles sent by the GUI (e.g., "30.55,15.22")
        try:
            theta1, theta2 = [float(i) for i in sent_angle_data.split(',')]

            # 2. Simulate noise (as done in your Arduino sketch)
            r = (random.randint(0, 100) / 1000.0) + 1  # Recreate noise factor

            theta1_noisy = theta1 * r
            theta2_noisy = theta2 * r

            # 3. Format the data string exactly as the Arduino would send it
            feedback_data = f"{theta1_noisy:.2f},{theta2_noisy:.2f}"

            # 4. Add the simulated final feedback to the queue for poll_arduino to read
            self._simulated_data.append(feedback_data)

        except Exception as e:
            print(f"SIMULATION ERROR: {e}")
            pass

    def send_data(self, data):
        if self.arduino and self.arduino.is_open:
            # Actual hardware send
            formatted_data = str(data) + "\n"
            self.arduino.write(formatted_data.encode('utf-8'))
            self.arduino.flush()
        else:
            # Simulation mode trigger
            self._simulate_send_receive(data)

    def read_data(self):
        if self.arduino and self.arduino.in_waiting > 0:
            # Actual hardware read
            return self.arduino.readline().decode('utf-8', errors='ignore').strip()
        elif not self.arduino and self._simulated_data:
            # Simulation mode read: pull one item from the front of the queue
            return self._simulated_data.pop(0)
        return None

    def close(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()



# takes user-inputted coordinates, runs them through the inverse kinematics function to calculate angles which are then
# sent to arduino, then uses the recieved angles from the arduino to plot the movement on computer graph
class GUI:
    #first part sets up computer interface
    def __init__(self, microcontroller: ArduinoCommunication):
        self.microcontroller = microcontroller


        # Main window
        self.root = Tk()
        self.root.title("Arduino Communication")
        self.root.geometry("600x800")

        # Frame
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid()

        # Label
        ttk.Label(self.frame, text="Enter coordinates in (X,Y) format:").grid(column=0, row=0, padx=5, pady=5)

        # Entry box for user input
        self.input_box = ttk.Entry(self.frame, width=20)
        self.input_box.grid(column=0, row=1, padx=5, pady=5)
        self.input_box.bind("<Return>", lambda event: self.send_to_arduino())


        # Send button
        self.send_button = ttk.Button(self.frame, text="Send", command=self.send_to_arduino)
        self.send_button.grid(column=1, row=1, padx=5, pady=5)

        # Feedback label
        self.feedback_label = ttk.Label(self.frame, text="")
        self.feedback_label.grid(column=0, row=2, columnspan=2, pady=10)

        # Received data display
        ttk.Label(self.frame, text="Received from Arduino:").grid(column=0, row=3, columnspan=2, pady=5)
        self.received_text = Text(self.frame, height=5, width=60)
        self.received_text.grid(column=0, row=4, columnspan=2, pady=5)

        # Matplotlib Figure for real-time plotting
        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Real-time Arduino Data")
        self.ax.set_xlabel("Position X (mm)")
        self.ax.set_ylabel("Position Y (mm)")
        self.line_target, = self.ax.plot([], [], color='blue')
        self.line_recieved, = self.ax.plot([], [], color='red')
        self.x_sent = []
        self.y_sent = []
        self.x_received = []
        self.y_received = []

        self.max_reach = L0 + L1 + L2
        self.ax.set_xlim(-self.max_reach, self.max_reach) # NEW limits
        self.ax.set_ylim(-self.max_reach, self.max_reach) # NEW limits
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

        # Start polling Arduino for incoming messages
        self.poll_arduino()

    # altered code, allows for the input of target coordinates rather than angles
    # second part sends instructions to arduino
    def send_to_arduino(self):

        #uses input and converts coordinates to angles which are then sent to arduino

        user_input = self.input_box.get().strip() #takes user input and removes any spaces from the sequence
        if not user_input:
            self.feedback_label.config(text="Please enter coordinates!") # message sent if textbox is empty
            return


        coord_pairs = re.findall(r'(\-?\d+)\s*,\s*(\-?\d+)(?:;|$)', user_input) #finds all coordinate pairs in user input and
                                                                                    # saves them as tuples names coord_pairs

        if not coord_pairs:
            self.feedback_label.config(text="Invalid format. Use X,Y;X2,Y2") # error message sent if coordinates not typed correctly
            return

        # clear previous target path data
        self.x_sent = []
        self.y_sent = []

        #for loop that checks through each pair of coordinates inputted and tries to convert input values to integers
        for x_str, y_str in coord_pairs:
            try:
                x_target = int(x_str)
                y_target = int(y_str)

                # uses the integer values to calculate theta angles required -> uses earlier ik function
                theta1, theta2 = calculate_ik(x_target, y_target)

                # checks that calculation was possible..
                if theta1 is not None and theta2 is not None:

                    # then sends the angle data to arduino
                    angle_data = f"{theta1:.2f},{theta2:.2f}"

                    self.microcontroller.send_data(angle_data)

                    # returns the actual angles for the target coordinates inputted
                    self.feedback_label.config(text=f"Sent: {angle_data} (Angles for {x_target},{y_target})")
                    

                    # stores the cartesian coordinates for plotting on computer (blue line)
                    self.x_sent.append(x_target)
                    self.y_sent.append(y_target)

                # if calculation wasn't possible for a set of coordinates, it returns below message
                else:
                    self.feedback_label.config(text=f"Skipped: ({x_target},{y_target}) unreachable.")

            # error message returned if non-number values inputted
            except ValueError:
                self.feedback_label.config(text="Error parsing number. Check format.")
                break

        self.input_box.delete(0, END)




    # from given code, checks arduino for data every 10ms - ?
    def poll_arduino(self):

        data = self.microcontroller.read_data()
        if data:
            self.received_text.insert(END, f"{data}\n")
            self.received_text.see(END)  #
            try:
                angle_data = [float(i) for i in data.split(',')]
                theta1_achieved = angle_data[0]
                theta2_achieved = angle_data[1]
            except:

                return


            x_achieved, y_achieved = calculate_fk(theta1_achieved, theta2_achieved)


            self.x_received.append(x_achieved)
            self.y_received.append(y_achieved)


            self.line_recieved.set_data(self.x_received, self.y_received)
            self.line_target.set_data(self.x_sent, self.y_sent)
            self.canvas.draw()

        # schedule next check for arduino data
        self.root.after(10, self.poll_arduino)

    def run(self):
        self.root.mainloop()

# Example usage - also from class code, links everything together
if __name__ == "__main__":
    micro = ArduinoCommunication(port="COM4", baudrate=115200)
    gui = GUI(micro)
    try:
        gui.run()
    finally:
        micro.close()

