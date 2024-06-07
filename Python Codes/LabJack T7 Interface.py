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
    
    FIO6: Trigger signal (digital output) of the ultrasonic sensor
    FIO7: Echo signal (digital input) of the ultrasonic sensor

"""
import time
import math
from labjack import ljm
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import keyboard


# Constant definition
PI = math.pi
SPEED_OF_SOUND = 343

# Variable definition
DT = 0.005

# Pin definition

PENDENCODER_PIN = "DIO0"
PENDENCODER_PINB = "DIO1"
MOTORENCODER_PIN = "DIO2"
MOTORENCODER_PINB = "DIO3"
MOTORPWM_PIN = "DIO4"
MOTORDIR_PIN = "DIO5"

ULTRASONICTRIG_PIN = "DIO6"
ULTRASONICECHO_PIN = "DIO7"


# Open connection to LabJack T7
handle = ljm.openS("T7", "USB", "ANY")

# =============================================================================
# LabJack Pin and Register Configuration
# =============================================================================

# Configure encoder pins
ljm.eWriteName(handle, PENDENCODER_PIN + "_EF_ENABLE", 0)
ljm.eWriteName(handle, PENDENCODER_PINB + "_EF_ENABLE", 0)
ljm.eWriteName(handle, PENDENCODER_PIN + "_EF_INDEX", 10)
ljm.eWriteName(handle, PENDENCODER_PINB + "_EF_INDEX", 10)
ljm.eWriteName(handle, PENDENCODER_PIN + "_EF_ENABLE", 1)
ljm.eWriteName(handle, PENDENCODER_PINB + "_EF_ENABLE", 1)

ljm.eWriteName(handle, MOTORENCODER_PIN + "_EF_ENABLE", 0)
ljm.eWriteName(handle, MOTORENCODER_PINB + "_EF_ENABLE", 0)
ljm.eWriteName(handle, MOTORENCODER_PIN + "_EF_INDEX", 10)
ljm.eWriteName(handle, MOTORENCODER_PINB + "_EF_INDEX", 10)
ljm.eWriteName(handle, MOTORENCODER_PIN + "_EF_ENABLE", 1)
ljm.eWriteName(handle, MOTORENCODER_PINB + "_EF_ENABLE", 1)


# Configure DC Motor direction pins (IN1 and IN2)
ljm.eWriteName(handle,MOTORDIR_PIN,0)

# Configure clock source for PWM output (DC Motor Speed Control)
ROLL_VALUE= 80000
ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 0)       # Disable the clock source
ljm.eWriteName(handle, "DIO_EF_CLOCK0_DIVISOR", 1)
ljm.eWriteName(handle, "DIO_EF_CLOCK0_ROLL_VALUE", ROLL_VALUE)
ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 1)
# Configure EF Channel Registers 
PWM_DUTY_CYCLE = 0   # Default to outputing 0% duty cycle
ljm.eWriteName(handle, MOTORPWM_PIN + "_EF_ENABLE", 0)   # Disable EF for initial configuration
ljm.eWriteName(handle, MOTORPWM_PIN + "_EF_INDEX", 0)    # Configure EF system for PWM
ljm.eWriteName(handle, MOTORPWM_PIN + "_EF_OPTIONS", 0)  # Configure clock source = Clock0
ljm.eWriteName(handle, MOTORPWM_PIN + "_EF_CONFIG_A", PWM_DUTY_CYCLE*ROLL_VALUE) # Duty cycle = 50%
ljm.eWriteName(handle, MOTORPWM_PIN + "_EF_ENABLE", 1)   # Enable EF, PWM outputted


# Define encoder objects blueprint
class Encoder:
    def __init__(self, resolution, angular_pos, angular_vel, encoder_pin):
        self.res = resolution # (pulse per revolution - PPR)
        self.angular_pos = angular_pos # (rad)
        self.angular_vel = angular_vel # (rad/s)
        self.pin = encoder_pin


# Instantiate encoder objects
pendEncoder = Encoder(2000,0,0,PENDENCODER_PIN + "_EF_READ_A_F_AND_RESET")
motorEncoder = Encoder(3800,0,0,MOTORENCODER_PIN + "_EF_READ_A_F_AND_RESET")


# =============================================================================
# Function Definition
# =============================================================================
def read_encoder(labjack_handle, dt, encoder_handle):
    # This function determine the angular velocity of encoder in RPM
    # DT = system time step.

    # Read encoder value
    val = ljm.eReadName(labjack_handle, encoder_handle.pin)/4

    # Calculate speed (assuming encoder produces 1600 velocitys per revolution)
    speed_rpm = val / (dt * encoder_handle.res) * 60            # (RPM)
    speed_rps = val / (dt * encoder_handle.res) * 2 * PI        # (rad/second)
    # speed = val

    # return [speed_rpm, speed_rps]
    return speed_rpm, speed_rps

def send_control_actions(labjack_handle, ctrl_action, vel_ctrl_pin, dir_pin):
    # u = control action.
    # vel_ctrl_pin = pin to send control to plant, probably DAC1
    # dir_pin = control the direction of the plant, probably DAC0
    
    # Limit control action within [-4.5, 4.5]
    ctrl_limit = 3.5
    ctrl_action = max(min(ctrl_action, ctrl_limit), -ctrl_limit)
    # Determine the required duty cycle of the PWM signal
    pwm_duty_cycle = abs(ctrl_action) / (4.5 - 0)
    # Set direction pin based on control action
    if ctrl_action > 0:
        ljm.eWriteName(labjack_handle,dir_pin,0)
    elif ctrl_action <= 0:
        ljm.eWriteName(labjack_handle,dir_pin,1)
    
    # Send control action to velocity control pin
    ljm.eWriteName(labjack_handle, vel_ctrl_pin + "_EF_CONFIG_A", pwm_duty_cycle*ROLL_VALUE)

    return ctrl_action
    
def send_trigger_pulse(labjack_handle,trig_pin):
    ljm.eWriteName(labjack_handle, trig_pin, 1)
    time.sleep(0.00001)
    ljm.eWriteName(labjack_handle, trig_pin, 0)

def measure_distance(labjack_handle, trig_pin, echo_pin):
    send_trigger_pulse(labjack_handle,trig_pin)
    # print("Pulse sent")
    time_out = 0.038

    pulse_start = time.time()
    pulse_timeout = pulse_start + time_out
    while ljm.eReadName(labjack_handle, echo_pin) == 0 and time.time() < pulse_timeout:
        pulse_start = time.time()
    # print(f"Echo high {pulse_start}")
    if time.time() >= pulse_timeout:
        return 0

    pulse_start = time.time()
    pulse_end = time.time()
    while ljm.eReadName(labjack_handle, echo_pin) == 1 and time.time() < pulse_timeout:
        pulse_end = time.time()
    # print(f"Echo low {pulse_end}")
    
    if time.time() >= pulse_timeout:
        return 0

    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * SPEED_OF_SOUND) / 2
    return distance

# =============================================================================
# Control Loop
# =============================================================================

# Setup live plotting
fig, ax = plt.subplots()
x_data, y_data = [], []
xDesired_data,yDesired_data = [], []
line, = ax.plot([], [], 'r-', label = 'Angular position')
lineDesired, = ax.plot([],[], 'b-', label = 'Desired position')

def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(-500, 500)
    # line.set_data([], [])
    # lineDesired.set_data([],[])
    return line, lineDesired

# Initial setup
last = time.time()
start_program = time.time()
errorPrev = 360
uPrev = 0

while (time.time() - start_program < 1):
    send_control_actions(handle, 0, MOTORPWM_PIN, MOTORDIR_PIN)

# Control loop and data collection function
def update(frame):
    global count, start_program, uPrev, errorPrev, last
    start = time.time()

    # ========== Obtain data from sensors ========== # 
    # Encoders
    [_,pendEncoder.angular_vel] = read_encoder(handle, DT, pendEncoder)
    [_,motorEncoder.angular_vel] = read_encoder(handle, DT, motorEncoder)

    # ========== Sensor Data Processing ========== # 
    pendEncoder.angular_pos += (pendEncoder.angular_vel * DT / PI) * 180
    motorEncoder.angular_pos += (motorEncoder.angular_vel * DT / PI) * 180

    cart_position = motorEncoder.angular_pos / 360 * 0.205

    # ========== Position Setpoint Test ========== # 
    current_time = time.time() - start_program
    angleDesired = 360 * np.sin(3 * current_time)
    # angleDesired = 360q

    # Control Action Calculation
    error = angleDesired - motorEncoder.angular_pos
    Kp = 0.050
    Kd = 0.005
    Ki = 0.001

    u = Kp * error
    # u = uPrev + (Kp + Ki * DT) * error - Kp * errorPrev
    # u = 0
    uPrev = u
    errorPrev = error

    if abs(u) < 0.05:
        u = 0


    # ========== Actuate the motor ========== # 
    controlActionOut = send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)

    # ========== Sensor Data Display ========== #
    print(f"{current_time:.5f} | "
        f"Vel_p (rad/s): {pendEncoder.angular_vel:5.1f} |"
        f"Theta_P (deg): {pendEncoder.angular_pos:6.1f} |"
        f"Vel_M (rad/s): {motorEncoder.angular_vel:5.1f} |"
        f"Theta_M (deg): {motorEncoder.angular_pos:6.1f} |"
        f"Desired (deg): {angleDesired:6.1f} | "
        f"Distance (m): {cart_position:5.2f} | "
        f"Error: {error:6.2f} | "
        f"Error Prev: {errorPrev:6.2f} | "
        f"Control In: {u:5.2f} | "
        f"Control Out: {controlActionOut:5.2f}")
    
    # speed += motorEncoder.angular_vel
    # print(f"Average DC Motor Speed: {speed/count:8.2f}")
    
    
    last = time.time()

    # Update data for plot
    x_data.append(current_time)
    y_data.append(motorEncoder.angular_pos)

    xDesired_data.append(current_time)
    yDesired_data.append(angleDesired)

    line.set_data(x_data, y_data)
    lineDesired.set_data(xDesired_data,yDesired_data)

    ax.set_xlim(current_time - 10, current_time)  # Adjust the x-axis to display the last 10 seconds
    ax.figure.canvas.draw()

    if keyboard.is_pressed("q"):
        u = 0
        controlActionOut = send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)
        print("Interupted! Current state log:")
        print(f"{current_time:.5f} | "
            f"Vel_p (rad/s): {pendEncoder.angular_vel:5.1f} |"
            f"Theta_P (deg): {pendEncoder.angular_pos:6.1f} |"
            f"Vel_M (rad/s): {motorEncoder.angular_vel:5.1f} |"
            f"Theta_M (deg): {motorEncoder.angular_pos:6.1f} |"
            f"Desired (deg): {angleDesired:6.1f} | "
            f"Distance (m): {cart_position:5.2f} | "
            f"Error: {error:6.2f} | "
            f"Error Prev: {errorPrev:6.2f} | "
            f"Control In: {u:5.2f} | "
            f"Control Out: {controlActionOut:5.2f}")
    
        time.sleep(2)
        ljm.close(handle)
        exit()

    return line, lineDesired

# Animate the live plot and run the control loop
ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 200), init_func=init, blit=True, interval=DT*1000)
plt.grid(which = "major", linewidth = 1)
plt.grid(which = "minor", linewidth = 0.2)
plt.legend(loc='upper left')
plt.minorticks_on()
plt.show()


