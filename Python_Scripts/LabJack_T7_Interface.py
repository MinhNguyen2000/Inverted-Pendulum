# -*- coding: utf-8 -*-
"""
Created on Fri Apr 26 12:55:00 2024

@author: Minh Nguyen

Pin Mapping:
    FIO0: Quadrature encoder channel A of the pendulum encoder
    FIO1: Quadrature encoder channel B of the pendulum encoder
    FIO2: Quadrature encoder channel A of the DC motor encoder
    FIO3: Quadrature encoder channel B of the DC motor encoder
    
    FIO4: PWM output for DC motor speed control
    FIO5: Direction output to contorl direction of the the motor on the MD10C board
    
    FIO6: Digital pin of the limit switch closer to the motor
    FIO7: Digital pin of the limit switch further away from the motor

"""
import time
import math
from labjack import ljm                     # LabJack interface
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import control
import keyboard                             # To obtain user input (key presses)
from pynput import keyboard                 # To obtain user input without needing root access

# Constant definition
PI = math.pi

# Pin definition
PENDENCODER_PIN     =   "DIO0"      # Pendulum quadrature encoder - channel A
PENDENCODER_PINB    =   "DIO1"      # Pendulum quadrature encoder - channel B
MOTORENCODER_PIN    =   "DIO2"      # Motor quadrature encoder - channel A
MOTORENCODER_PINB   =   "DIO3"      # Motor quadrature encoder - channel B
MOTORPWM_PIN        =   "DIO4"      # Motor speed control (PWM) pin
MOTORDIR_PIN        =   "DIO5"      # Motor direciton control pin
LIMIT_SWITCH1_PIN   =   "DIO6"      # The limit switch closer to the motor
LIMIT_SWITCH2_PIN   =   "DIO7"      # The limit switch further from the motor

ROLL_VALUE = 80000                   # Determine the timer frequency for motor PWM (speed) control

# Variable definition
DT = 0.001                          # Sampling time
center_position = 1.25/2-0.04       # Center position of the cart
angle_setup_threshold = 10          # Setup threshold for balancing the pendulum from top position (setup within this range from 180deg)
angle_balance_threshold = 30        # Threshold for active control
cart_position_threshold = 0.50      # Threshold for active control. If difference between cart position and desired position outside this range, stop control.

# System control state definition
current_ctrl_state = 'idle'

# Physical system parameters
m = 0.113/3         # Pendulum mass (kg)
M = 2.465           # Cart mass (kg)
L = 0.15            # Pendulum length (m)
g = -9.81           
d = 0.0001          # Damping coefficient (N*s/m) between cart and rail
c = 75            # Force-Voltage coefficient (N/V) relating the voltage to the motor and the outputted force

# Linearized system model (Jacobian of the motion model at theta = pi, theta_dot = 0)
A = np.array([[0,       1,          0,                  0],
              [0,       -d/M,       m*g/M,              0],
              [0,       0,          0,                  1],
              [0,       -d/(L*M),   -(M+m)*g/(L*M),     0]])
B = np.array([[0,       1/M,        0,                  1/(L*M)]]).reshape(-1,1)

# Assume full-state feedback
C = np.eye(4)
D = np.zeros(4).reshape(-1,1)

sys_ss_cont = control.ss(A, B, C, D)
sys_ss_disc = control.c2d(sys_ss_cont,DT) 

Ad = sys_ss_disc.A
Bd = sys_ss_disc.B
Q = np.eye(A.shape[1])
Q[0,0] = 10000
Q[2,2] = 50
R = np.array([1])
K,_,_ = control.dlqr(Ad,Bd,Q,R)

# =============================================================================
# LabJack Initialization and Pin Configuration Functions
# =============================================================================
def initialize_labjack():
    '''Function for connecting to the LabJack T7'''
    handle = ljm.openS("T7", "USB", "ANY")
    print("LabJack T7 connected")
    return handle

class Encoder:
    def __init__(self, name, resolution, encoder_pin, encoder_pinb, angular_pos = 0, angular_vel = 0):
        self.name = name
        self.res = resolution                           # (pulse per revolution - PPR)
        self.encoder_pin = encoder_pin                  # the quadrature encoder read pin
        self.encoder_pinb = encoder_pinb

        self.angular_pos_prev = 0
        self.angular_pos = angular_pos                  # angular position (rad) - initialized as 0
        self.angular_pos_deg = self.angular_pos / PI * 180  # angular position (deg)
        self.angular_vel = angular_vel                  # angular velovity (rad/s) - initialized as 0
        self.angular_vel_rpm = self.angular_vel / (2 * PI) * 60
        

    def initialize_encoder(self, labjack_handle):
        '''Function to configure the pins of the LabJack T7 to enable quadrature encoder reading
        
        Parameters:
        - labjack_handle: the LabJack handle created from labjack initialization
        - resolution: the encoder resolution
        - encoder_pin: quadrature encoder pin A
        - encoder_pinb: quadrature encoder pin B
        '''

        start_time = time.time()

        # Configure encoder pins
        ljm.eWriteName(labjack_handle, self.encoder_pin + "_EF_ENABLE", 0)
        ljm.eWriteName(labjack_handle, self.encoder_pinb + "_EF_ENABLE", 0)
        ljm.eWriteName(labjack_handle, self.encoder_pin + "_EF_INDEX", 10)
        ljm.eWriteName(labjack_handle, self.encoder_pinb + "_EF_INDEX", 10)
        ljm.eWriteName(labjack_handle, self.encoder_pin + "_EF_ENABLE", 1)
        ljm.eWriteName(labjack_handle, self.encoder_pinb + "_EF_ENABLE", 1)

        # pendEncoder = Encoder(2000,PENDENCODER_PIN + "_EF_READ_A_F_AND_RESET")
        # motorEncoder = Encoder(3800,MOTORENCODER_PIN + "_EF_READ_A_F_AND_RESET")
        
        return f'The {self.name} encoder is initialized in {time.time()-start_time}s'

    def read_encoder(self, labjack_handle, dt):
        ''' This function calculates the current angular velocity of encoder in RPM and rad/s
        Parameters:
        - labjack_handle: the handle for the LabJack object
        - encoder: the encoder object to read
        - dt: sampling time

        Output:
        - speed_rpm: 
        '''
        # Read encoder value
        val = ljm.eReadName(labjack_handle, self.encoder_pin + "_EF_READ_A_F_AND_RESET")/4

        # Calculate speed
        speed_rpm = val / (dt * self.res) * 60            # (RPM)
        speed_rps = val / (dt * self.res) * 2 * PI        # (rad/second)

        # return [speed_rpm, speed_rps]
        return speed_rpm, speed_rps

    def reset_encoder(self, labjack_handle):
        ljm.eReadName(labjack_handle, self.encoder_pin + "_EF_READ_A_F_AND_RESET")              # Reset the encoder values
    
    def update_state(self, labjack_handle, dt=DT):
        # Read quadrature encoder value
        val = ljm.eReadName(labjack_handle, self.encoder_pin + "_EF_READ_A_F")              # Reading the encoder pulses
        
        self.angular_pos_prev = self.angular_pos
        self.angular_pos = (val/4) / self.res * (2 * PI)                                    # Current position (rad)
        self.angular_pos_deg = self.angular_pos / PI * 180                                  # Current position (degree)
        self.angular_vel = (self.angular_pos - self.angular_pos_prev) / dt                  # Velocity (rad/s)
        self.angular_vel_rpm = self.angular_vel / (2 * PI) * 60                             # Velocity (rpm)

class Motor:
    def __init__(self, motorpwm_pin, motordir_pin):
        self.motorpwm_pin = motorpwm_pin
        self.motordir_pin = motordir_pin

    def intialize_motor(self, labjack_handle):
        '''Function to initialize the motor by first configuring the PWM clock then PWM cycle
        
        Parameters:
        - handle: the LabJack handle created from labjack initialization
        - motorpwm_pin:
        - motordir_pin:
        
        Output
        - motor: the motor object of class Motor
        '''

        start_time = time.time()
        # Instantiate the motor object of class Motor
        motor = Motor(self.motorpwm_pin,self.motordir_pin)

        # LabJack - Configure DC Motor direction pins to one direction
        ljm.eWriteName(labjack_handle,self.motordir_pin,0)

        # LabJack - Configure clock source for PWM output (DC Motor Speed Control)
        ljm.eWriteName(labjack_handle, "DIO_EF_CLOCK0_ENABLE", 0)               # Disable the clock source (Clock 0)
        ljm.eWriteName(labjack_handle, "DIO_EF_CLOCK0_DIVISOR", 1)              # Set the clock divisor as 1
        ljm.eWriteName(labjack_handle, "DIO_EF_CLOCK0_ROLL_VALUE", ROLL_VALUE)  # Set the roll value
        ljm.eWriteName(labjack_handle, "DIO_EF_CLOCK0_ENABLE", 1)               # Re-enable the clock source
        
        # LabJack - Configure EF Channel Registers 
        PWM_DUTY_CYCLE = 0   # Default to outputing 0% duty cycle
        ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_ENABLE", 0)          # Disable EF for initial configuration
        ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_INDEX", 0)           # Configure EF system for PWM
        ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_OPTIONS", 0)         # Configure clock source = Clock0
        ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_CONFIG_A", PWM_DUTY_CYCLE*ROLL_VALUE) # Duty cycle = 0%
        ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_ENABLE", 1)          # Enable EF, PWM outputted

        print(f'Motor was initialized in {time.time() - start_time}s')
        return motor
    
    def send_control_actions(self, labjack_handle, ctrl_action :float):
        '''Function to actuate the motor
        
        Parameters:
        - labjack_handle: the handle for the LabJack object
        - ctrl_action: the control action to send to the motor (bounded)
        '''

        # Limit control action
        ctrl_limit = 3.5
        ctrl_action = max(min(ctrl_action, ctrl_limit), -ctrl_limit)

        # Determine the required duty cycle of the PWM signal
        pwm_duty_cycle = abs(ctrl_action) / (4.5 - 0)
        try:
            # Set direction pin based on control action
            if ctrl_action > 0:
                ljm.eWriteName(labjack_handle,self.motordir_pin,0)
            elif ctrl_action <= 0:
                ljm.eWriteName(labjack_handle,self.motordir_pin,1)
            
            # Send control action to velocity control pin
            ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_CONFIG_A", pwm_duty_cycle*ROLL_VALUE)
        except:
            print("Error")
        return ctrl_action

class Cart:
    def __init__(self, position=0, velocity=0):
        self.pos = position
        self.vel = velocity

# =============================================================================
# Low-level Sensor and Actuator Control Functions
# =============================================================================
def read_limitswitches(labjack_handle, limitswitch_pins):
    '''
    Function for reading the digital output of the limit switches
    '''
    val = []
    for pin in limitswitch_pins:
        val.append(ljm.eReadName(labjack_handle,pin))
    return val

def read_and_process_sensors(labjack_handle, pendEncoder:Encoder, motorEncoder:Encoder, cart:Cart, dt=DT):
    """
    Reads and processes data from pendulum and motor encoders to update the current states of the system.
    Expected Behaviour:
        The angular position and velocity of the pendEncoder and motorEncoder objects are updated
        Returns a dictionary of the important states (such as encoder position and speed, and cart position and speed)
    
    Parameters:
        handle: LabJack handle for communication.
        pendEncoder: Encoder object for the pendulum.
        motorEncoder: Encoder object for the motor.
        dt: Time step for reading encoders.
        
    Returns:
        dict: Processed sensor data including angular velocities, positions, and cart position.
    """
    # # Old method with read_encoder
    # # Read encoder data
    # [_, pendEncoder.angular_vel] = pendEncoder.read_encoder(labjack_handle, dt=dt)
    # [_, motorEncoder.angular_vel] = motorEncoder.read_encoder(labjack_handle, dt=dt)
    
    # # Calculate encoder angular positions
    # pendEncoder.angular_pos += (pendEncoder.angular_vel * dt / PI) * 180
    # motorEncoder.angular_pos += (motorEncoder.angular_vel * dt / PI) * 180

    pendEncoder.update_state(labjack_handle)
    motorEncoder.update_state(labjack_handle)

    # Calculate cart position and speed
    motorPulley_diameter = 65.3e-3                                  # Diameter of the motor pulley (m)
    motorPulley_circ = PI * motorPulley_diameter                    # Circumference of the motor pulley (m)
    cart.pos = motorEncoder.angular_pos / (2 * PI) * motorPulley_circ
    cart.vel = motorEncoder.angular_vel / (2 * PI) * motorPulley_circ   # rad/s * (pulley_circumference (m) / 2pi rads)

    # Return processed data
    return {
        "P_angular_vel": pendEncoder.angular_vel,
        "P_angular_pos": pendEncoder.angular_pos,
        "M_angular_vel": motorEncoder.angular_vel,
        "M_angular_pos": motorEncoder.angular_pos,
        "cart_pos": cart.pos,
        "cart_vel": cart.vel,
    }

# =============================================================================
# User Interaction Functions
# =============================================================================

def get_user_input():
    inputs = {
        "1": "calibration",
        "2": "centering",
        "3": "zeroing",
        "4": "balance",
        "5": "swing up",
        "9": "state report"
    }

    print("\nPossible states for the inverted pendulum system")
    for key, value in inputs.items():
        print(f"{key}. {value}")    
    while True:
        user_input = input("Enter the number corresponding to your choice: ").strip()
        if user_input in inputs:
            return inputs[user_input]
        else: 
            print("Invalid input. please enter a number from the list")

# Global variables for user input
user_ready = False
within_range = False

def on_key_press(key):
    """
    Handle key presses to set the system state.
    """
    global user_ready, current_ctrl_state
    try:
        if key.char == 's' and within_range:  # 's' starts balancing only when within range
            user_ready = True
            print("[INFO] User confirmed start of balancing.")
        if key.char == 'q':
            print('[INFO] User-specified quit to idle mode')
            current_ctrl_state = 'idle'
    except AttributeError:
        pass  # Handle special keys like Shift, etc.

# =============================================================================
# SystemHistory Class for Plotting
# =============================================================================
class SystemHistory:
    def __init__(self):
        # Initialize a dictionary where keys are the sensor states, and values are lists to hold the historical data.
        self.history = {
            'time': [],
            'pendulum_pos': [],
            'pendulum_vel': [],
            'cart_pos': [],
            'cart_vel': [],
            'control': [],
            # You can add more keys as necessary for other sensor states.
        }

    def append_state(self, pendulum_data: Encoder, cart_data: Cart, control_action, timestamp):
        # Append the values from the sensor_data to the respective lists in the dictionary
        self.history['time'].append(timestamp)
        self.history['pendulum_pos'].append(pendulum_data.angular_pos)
        self.history['pendulum_vel'].append(pendulum_data.angular_vel)
        self.history['cart_pos'].append(cart_data.pos)
        self.history['cart_vel'].append(cart_data.vel)
        self.history['control'].append(control_action)

    def get_history(self):
        return self.history

    def reset_history(self):
        # Clear the history if needed
        self.history = {key: [] for key in self.history}

system_history = SystemHistory()

# =============================================================================
# State Functions
# =============================================================================
def calibration_process(labjack_handle, 
                        pendEncoder: Encoder, motorEncoder: Encoder, 
                        motor: Motor, cart: Cart, limit_switch_pins):
    """
    Calibration function to move the cart to the limit switch(es) and center it. After this procedure, the current state goes to "idle"
    
    Parameters:
        handle: LabJack handle for communication.
        pendEncoder: Encoder object for the pendulum.
        motorEncoder: Encoder object for the motor.
        motor: Motor object to control the cart.
        limit_switch_pins: List of digital pins connected to limit switches.

    Return:
        "idle" as the current state
    """
    
    listener = keyboard.Listener(on_press=on_key_press)
    listener.start() 
    print("Starting calibration...")

    # # Move the cart to the first limit switch
    # print("Moving to the first limit switch...")
    # motor.send_control_actions(labjack_handle, +1.0)

    # while True:
    #     # Read limit switch states
    #     switch_states = read_limitswitches(labjack_handle, limit_switch_pins)

    #     # Exit loop if the first limit switch is press (value = 0.0)
    #     if not(switch_states[1]):    # Limit switch further from the motor
    #         print("First limit switch triggered.")
    #         motor.send_control_actions(labjack_handle,0)
    #         break

    #     # # Read and process sensor data for debugging or monitoring
    #     sensor_data = read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, dt=DT)
    #     print(f"Cart position: {sensor_data['cart_pos']:.2f}m | Switches: {switch_states}")

    # Move the cart to the second limit switch    
    print("Moving toward the limit switch...")
    motor.send_control_actions(labjack_handle, -1.0)
    
    while current_ctrl_state == 'calibration':
        # Read limit switch states
        switch_states = read_limitswitches(labjack_handle, limit_switch_pins)

        # Exit loop if the first limit switch is pressed (value = 0.0)
        if not(switch_states[0]):  # Limit switch closer to the motor
            print("Limit switch triggered.")
            motor.send_control_actions(labjack_handle,0)
            motorEncoder.angular_pos = 0                            # Reset the motor encoder position
            motorEncoder.angular_pos_deg = 0
            motorEncoder.reset_encoder(labjack_handle)
            cart.pos = 0
            break

        # # Read and process sensor data for debugging or monitoring
        read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, cart, dt=DT)
        print(f"Cart position: {cart.pos:.2f} m | Switches: {switch_states}",end = ' \r')

    # Move the cart to the middle position
    motor.send_control_actions(labjack_handle, +1.0)

    while current_ctrl_state == 'calibration':
        read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, cart, dt=DT)
        print(f"Cart position: {cart.pos:.2f} m", end = ' \r')
        # Stop if the cart is at the center position
        if abs(cart.pos - center_position) < 0.01:   # Example threshold
            print("Cart centered.")
            motor.send_control_actions(labjack_handle,0)
            motorEncoder.angular_pos = 0                            # Reset the motor encoder position (middle = 0)
            motorEncoder.reset_encoder(labjack_handle)
            break
    
        # print(f"Vel_p (rad/s): {sensor_data['P_angular_vel']:6.1f} |"
        #                 f"Theta_P (deg): {sensor_data['P_angular_pos']:6.1f} |"
        #                 f"Vel_M (rad/s): {sensor_data['M_angular_vel']:6.1f} |"
        #                 f"Theta_M (deg): {sensor_data['M_angular_pos']:6.1f} |"
        #                 f"Distance (m): {sensor_data['cart_pos']:5.2f} | "
        #                 f"Velocity (m/s): {sensor_data['cart_vel']:5.5f}"
        #             )
    print("Calibration complete.")

    listener.stop()

    return 'idle'

def zeroing(labjack_handle, pendEncoder: Encoder, motorEncoder: Encoder, cart: Cart):
    global current_ctrl_state
    read_and_process_sensors(labjack_handle,pendEncoder,motorEncoder,cart)
    pendEncoder.angular_pos_prev = 0
    pendEncoder.angular_pos = 0
    pendEncoder.angular_pos_deg = 0
    pendEncoder.angular_vel = 0
    pendEncoder.angular_vel_rpm = 0
    pendEncoder.reset_encoder(labjack_handle)

    motorEncoder.angular_pos_prev = 0
    motorEncoder.angular_pos = 0
    motorEncoder.angular_pos_deg = 0
    motorEncoder.angular_vel = 0
    motorEncoder.angular_vel_rpm = 0
    motorEncoder.reset_encoder(labjack_handle)

    cart.pos = 0
    cart.vel = 0

    current_ctrl_state = 'idle'
    print("\nSystem states are reset")
    return

def centering(labjack_handle, pendEncoder: Encoder, motorEncoder: Encoder, cart: Cart, motor: Motor, dt = DT):
    global current_ctrl_state
    read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, cart, dt=DT)
    motor.send_control_actions(labjack_handle, - np.sign(cart.pos))

    while True:
        read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, cart, dt=DT)

        # Stop if the cart is at the center position
        if abs(cart.pos) < 0.01:   # Example threshold
            print("Cart centered.")
            motor.send_control_actions(labjack_handle,0)
            motorEncoder.angular_pos = 0                            # Reset the motor encoder position (middle = 0)
            break
    
        # print(f"Vel_p (rad/s): {sensor_data['P_angular_vel']:6.1f} |"
        #                 f"Theta_P (deg): {sensor_data['P_angular_pos']:6.1f} |"
        #                 f"Vel_M (rad/s): {sensor_data['M_angular_vel']:6.1f} |"
        #                 f"Theta_M (deg): {sensor_data['M_angular_pos']:6.1f} |"
        #                 f"Distance (m): {sensor_data['cart_pos']:5.2f} | "
        #                 f"Velocity (m/s): {sensor_data['cart_vel']:5.5f}"
        #             )

    current_ctrl_state = 'idle'
    print("Centering process complete.")

def balance_process(labjack_handle, pendEncoder: Encoder, motorEncoder: Encoder, motor: Motor, cart: Cart, limit_switch_pins, system_history: SystemHistory, dt=DT):
    global within_range, user_ready
    print("Starting balancing state from the top position")

    print(f"Please move the pendulum to within {angle_setup_threshold} degrees of the top position")
    print("Once the desired starting position is reached, press 's' to start balancing")
    listener = keyboard.Listener(on_press=on_key_press)
    listener.start()        # Start the listener to listen to key press ('s' to start or 'q' to quit)

    # Waiting for the user to manually bring the pendulum up and press 's'
    while not user_ready and current_ctrl_state == 'balance':
        read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, cart)
        
        is_in_range = abs(abs(pendEncoder.angular_pos_deg) - 180) <= angle_setup_threshold

        # Check if the pendulum is within the setup threshold
        if is_in_range:
            if not within_range:    # If the pendulum was brought into range but not was not previously within range
                print("Pendulum is within the acceptable range. Hold it steady.")
                within_range = True
        else:
            if within_range:        # if tthe pendulum was brought outside of range but was previously within range
                # Notify the user when the pendulum drops outside the acceptable range
                print("Pendulum is outside the acceptable range! Please adjust it back.")
            within_range = False

        time.sleep(0.1)  # Allow time for user to respond
    listener.stop()

    user_ready = False  # Reset this flag for future balances
    print("Beginning control process.")
    start_time_balance = time.time()
    while current_ctrl_state == 'balance':
        read_and_process_sensors(labjack_handle, pendEncoder, motorEncoder, cart, dt=DT)
        
        current_state = np.array([[cart.pos, cart.vel, pendEncoder.angular_pos_deg, pendEncoder.angular_vel]]).reshape(-1,1)
        desired_state = np.array([[0, 0, 180, 0]]).reshape(-1,1)
        # Active control algorithm goes here
        
        # Control output (force)
        u_F = -K @ (current_state-desired_state)
        # Control output (voltage to the motor)
        u_V = u_F/c
        u_V = float(u_V)
        # Logging the system state
        current_time_balance = time.time() - start_time_balance
        # system_history.append_state(pendEncoder,cart,
        #                             control_action=u,
        #                             timestamp=current_time_balance)
        
        # Print the control ouput 
        u_V_out = motor.send_control_actions(labjack_handle,u_V)
        print(f"Sent {u_V_out:6.3f}V to the motor | "
              f"Pos_c (m): {cart.pos:6.3f} | "
              f"Vel_c (m/s): {cart.vel:6.3f} | "
              f"Vel_p (rad/s): {pendEncoder.angular_vel:6.2f} | "
              f"Theta_P (deg): {pendEncoder.angular_pos_deg:6.2f} |", end = ' \r'
              )

        # Emergency stop cases
        if abs(pendEncoder.angular_pos_deg - 180) > angle_balance_threshold:
            print("Control failed - Exceed pendulum angle limit")
            # current_ctrl_state = 'idle'
            return 'idle'

        if abs(cart.pos) > cart_position_threshold:
            print("Control failed - Exceed cart position limit")
            # current_ctrl_state = 'idle'
            return 'idle'
        
        if current_time_balance >= 60:
            print("Controlled successfully for 10 seconds. You can plot the system history")
            # current_ctrl_state = 'idle'
            return 'idle'

    print("Control process done")
    return "idle"

# For plotting the history
def plotting(system_history: SystemHistory):
    history = system_history.get_history()
    pass

def stop_control(labjack_handle, motor: Motor):
    # Send zero volt to the motor
    motor.send_control_actions(labjack_handle, 0)
    print('Motor control stopped (Idle Mode)')

def stop_program(labjack_handle, motor: Motor):
    # Send zero volt to the motor
    motor.send_control_actions(labjack_handle, 0)
    print('Motor control stopped (Program stopped)')

    time.sleep(2)
    ljm.close(labjack_handle)
    print('LabJack disconnected')

    exit()
        
    

# =============================================================================
# Main Program)
# =============================================================================

def main():
    handle = initialize_labjack()

    # Instantiate and initialize the sensors/actuators
    pendEncoder = Encoder(name="pendEncoder",
                          resolution=2000,
                          encoder_pin=PENDENCODER_PIN,
                          encoder_pinb=PENDENCODER_PINB)
    pendEncoder.initialize_encoder(handle)

    motorEncoder = Encoder(name="motorEncoder",
                           resolution=3800,
                           encoder_pin=MOTORENCODER_PIN,
                           encoder_pinb=MOTORENCODER_PINB)
    motorEncoder.initialize_encoder(handle)

    motor = Motor(motorpwm_pin=MOTORPWM_PIN,
                  motordir_pin=MOTORDIR_PIN)
    motor.intialize_motor(handle)

    cart = Cart()

    global current_ctrl_state
    current_ctrl_state = 'idle'

    # Main control loop
    try:
        while True:
            match current_ctrl_state:
                case 'idle':            # if not in the process of anything, ask user for input
                    stop_control(handle,motor)
                    current_ctrl_state = get_user_input()
                    
                case 'calibration':
                    listener = keyboard.Listener(on_press=on_key_press)
                    listener.start()
                    current_ctrl_state = calibration_process(labjack_handle=handle, 
                                        pendEncoder=pendEncoder, motorEncoder=motorEncoder,
                                        motor=motor, cart=cart,
                                        limit_switch_pins=[LIMIT_SWITCH1_PIN,LIMIT_SWITCH2_PIN])
                    listener.stop()
                case 'centering':
                    centering(handle, pendEncoder, motorEncoder, cart, motor)
                    pass
                case 'zeroing':
                    zeroing(handle, pendEncoder, motorEncoder,cart)
                    pass
                case 'balance':
                    print(f"Control state 0: {current_ctrl_state}")
                    current_ctrl_state = balance_process(labjack_handle=handle, 
                                        pendEncoder=pendEncoder, motorEncoder=motorEncoder,
                                        motor=motor, cart=cart,
                                        limit_switch_pins=[LIMIT_SWITCH1_PIN,LIMIT_SWITCH2_PIN],
                                        system_history = system_history)
                    # current_ctrl_state = state
                case 'swing up':
                    read_and_process_sensors(handle, pendEncoder, motorEncoder, cart, dt=DT)
                    
                    pass
                case 'state report':
                    listener = keyboard.Listener(on_press=on_key_press)
                    listener.start()

                    # Display sensor data
                    while current_ctrl_state == 'state report':
                        read_and_process_sensors(handle, pendEncoder, motorEncoder, cart, dt=DT)
                        print(f"Vel_p (rad/s): {pendEncoder.angular_vel:6.1f} |"
                            f"Theta_P (deg): {pendEncoder.angular_pos_deg:6.1f} |"
                            f"Vel_M (rad/s): {motorEncoder.angular_vel:6.1f} |"
                            f"Theta_M (deg): {motorEncoder.angular_pos_deg:6.1f} |"
                            f"Distance (m): {cart.pos:5.2f} | "
                            f"Velocity (m/s): {cart.vel:5.2f}", end = ' \r', flush = True
                        )
                        
                    listener.stop()
                    
                    current_ctrl_state = "idle"
    except KeyboardInterrupt:
        stop_program(handle,motor)
        pass

    finally: 
        pass

if __name__ == "__main__":
    main()
