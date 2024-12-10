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
from labjack import ljm
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import keyboard


# Constant definition
PI = math.pi
SPEED_OF_SOUND = 343

# Variable definition
DT = 0.0025

# Pin definition
PENDENCODER_PIN     =   "DIO0"
PENDENCODER_PINB    =   "DIO1"
MOTORENCODER_PIN    =   "DIO2"
MOTORENCODER_PINB   =   "DIO3"
MOTORPWM_PIN        =   "DIO4"
MOTORDIR_PIN        =   "DIO5"
LIMIT_SWITCH1_PIN   =   "DIO6"      # The limit switch closer to the motor
LIMIT_SWITCH2_PIN   =   "DIO7"      # The limit switch further from the motor

# State definition
STATE_CURRENT = "startup"           
STOP_MSG = ""

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


# Configure DC Motor direction pins to one direction
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
ljm.eWriteName(handle, MOTORPWM_PIN + "_EF_CONFIG_A", PWM_DUTY_CYCLE*ROLL_VALUE) # Duty cycle = 0%
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
def read_limitswitch(labjack_handle, limitswitch_pin):
    val = ljm.eReadName(labjack_handle,limitswitch_pin)
    return val

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

def stop_program():
    u = 0
    controlActionOut = send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)
    print(f"{STOP_MSG}! Current state log:")
    print(f"{current_time:.5f} | "
        f"Switch: {limitSwitch_state:1.0f} | "
        f"Vel_p (rad/s): {pendEncoder.angular_vel:6.1f} |"
        f"Theta_P (deg): {pendEncoder.angular_pos:6.1f} |"
        f"Vel_M (rad/s): {motorEncoder.angular_vel:6.1f} |"
        f"Theta_M (deg): {motorEncoder.angular_pos:6.1f} |"
        f"Desired (deg): {angleDesired:6.1f} | "
        f"Distance (m): {cart_position:5.2f} | "
        # f"Error: {error:6.2f} | "
        # f"Error Prev: {errorPrev:6.2f} | "
        f"Control In: {u:5.2f} | "
        f"Control Out: {controlActionOut:5.2f}")

    time.sleep(2)
    ljm.close(handle)
    exit()

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

# Control loop and data collection function.
# This function is used with an animation.FuncAnimation() function to collect the data, create a live plot, and actuate the motor
def update(frame):
    global count, start_program, uPrev, errorPrev, last
    global current_time,limitSwitch_state, angleDesired,cart_position
    global STATE_CURRENT, STOP_MSG
    start = time.time()

    # ========== Obtain data from sensors ========== # 
    # Limit Switch
    limitSwitch_state = read_limitswitch(handle,LIMIT_SWITCH1_PIN)

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
        f"Switch: {limitSwitch_state:1.0f} | "
        f"Vel_p (rad/s): {pendEncoder.angular_vel:6.1f} |"
        f"Theta_P (deg): {pendEncoder.angular_pos:6.1f} |"
        f"Vel_M (rad/s): {motorEncoder.angular_vel:6.1f} |"
        f"Theta_M (deg): {motorEncoder.angular_pos:6.1f} |"
        f"Desired (deg): {angleDesired:6.1f} | "
        f"Distance (m): {cart_position:5.2f} | "
        # f"Error: {error:6.2f} | "
        # f"Error Prev: {errorPrev:6.2f} | "
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

    ax.set_xlim(current_time - 20, current_time)  # Adjust the x-axis to display the last 10 seconds
    ax.figure.canvas.draw()

    if limitSwitch_state == 0:
        STATE_CURRENT = "Stopped"
        STOP_MSG = "Limit switch interruption"
        stop_program()

    if keyboard.is_pressed("q"):
        STATE_CURRENT = "Stopped"
        STOP_MSG = "Keyboard interruption"
        stop_program()

        

    return line, lineDesired

# =============================================================================
# Control Loop
# =============================================================================


ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 200), init_func=init, blit=True, interval=DT*1000)
print('Hey')
plt.grid(which = "major", linewidth = 1)
plt.grid(which = "minor", linewidth = 0.2)
plt.legend(loc='upper left')
plt.minorticks_on()
plt.show()


