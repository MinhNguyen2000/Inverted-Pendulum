"""
Created on May 31, 2024

@author: Minh Nguyen

Pin Mapping:
    FIO0: Quadrature encoder channel A of the pendulum encoder
    FIO1: Quadrature encoder channel B of the pendulum encoder
    FIO2: Quadrature encoder channel A of the DC motor ePncoder
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
MOTORENCODER_PIN = "DIO4"
MOTORENCODER_PINB = "DIO5"
MOTORPWM_PIN = "DIO7"
MOTORDIR_PIN = "DIO6"

# Open connection to LabJack T4
handle = ljm.openS("T4", "USB", "ANY")

# =============================================================================
# LabJack Pin and Register Configuration
# =============================================================================

# Configure encoder pins
ljm.eWriteName(handle, MOTORENCODER_PIN + "_EF_ENABLE", 0)
ljm.eWriteName(handle, MOTORENCODER_PINB + "_EF_ENABLE", 0)
ljm.eWriteName(handle, MOTORENCODER_PIN + "_EF_INDEX", 10)
ljm.eWriteName(handle, MOTORENCODER_PINB + "_EF_INDEX", 10)
ljm.eWriteName(handle, MOTORENCODER_PIN + "_EF_ENABLE", 1)
ljm.eWriteName(handle, MOTORENCODER_PINB + "_EF_ENABLE", 1)

# Configure DC Motor direction pins (IN1 and IN2)
ljm.eWriteName(handle,MOTORDIR_PIN,0)

# Configure clock source for PWM output (DC Motor Speed Control)
ROLL_VALUE= 80000                                       # Roll value of T4 and T7 is 80000 / (divisor * desired frequency)
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


# =============================================================================
# Hardware Instantiation
# =============================================================================

# Define encoder objects blueprint
class Encoder:
    def __init__(self, resolution, angular_pos, angular_vel, encoder_pin):
        self.res = resolution # (pulse per revolution - PPR)
        self.angular_pos = angular_pos # (rad)
        self.angular_vel = angular_vel # (rad/s)
        self.pin = encoder_pin

# Instantiate encoder objects
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

    # return [speed_rpm, speed_rps]q
    return speed_rpm, speed_rps

def send_control_actions(labjack_handle, ctrl_action, vel_ctrl_pin, dir_pin):
    # u = control action.
    # vel_ctrl_pin = pin to send control to plant, probably DAC1
    # dir_pin = control the direction of the plant, probably DAC0
    
    # Limit control action within [-4.5, 4.5]
    max_ctrl = 3.8
    ctrl_action = max(min(ctrl_action, 4), -4)
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

# last = time.time()
# start_program = time.time()

# while (time.time() - start_program < 1):
#     send_control_actions(handle, 0, MOTORPWM_PIN, MOTORDIR_PIN)

# angleDesired = 720
# errorPrev = angleDesired
# uPrev = 0

# while True:
#     start = time.time()
#     # ========== Obtain data from sensors ========== # 
#     # Encoders
#     [_,motorEncoder.angular_vel] = read_encoder(handle, DT, motorEncoder)

#     # ========== Sensor Data Processing ========== # 
#     motorEncoder.angular_pos += (motorEncoder.angular_vel * DT / PI) * 180

#     # ========== Control Action Calculation ========== #
#     currentTime = time.time() - start_program
    
#     # u = 3 * math.sin(currentTime)
    
#     error = angleDesired - motorEncoder.angular_pos
#     Kp = 0.0025
#     Kd = 0.00005
#     Ki = 0.01

#     # PD Controller
#     # u = Kp * error + Kd * (error - errorPrev)

#     # PI Controller
#     u = uPrev + (Kp + Ki * DT) * error - Kp * errorPrev
#     uPrev = u
#     errorPrev = error
#     # u = 1

#     if abs(u) < 0.1:
#         u = 0
#     # ========== System Actuation ========== #
#     controlAction = send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)

#     # ========== Sensor Data Display ========== #
#     print(f"{start - last:.5f} | "
#         f"Vel_M (rad/s): {motorEncoder.angular_vel:8.2f} |"
#         f"Theta_M (deg): {motorEncoder.angular_pos:8.2f} |"
#         f"Control in: {u:8.3f} | "
#         f"Control out: {controlAction:8.3f}")
    
#     # speed += motorEncoder.angular_vel
#     # print(f"Average DC Motor Speed: {speed/count:8.2f}")
    
#     # ========== Sensor Data Visualization ========== #

#     # Ensure each loop takes the same amount of time as the sampling time
#     end = time.time()
#     last = end
#     # print("Elapsed time: {:.3e}".format(end-start))
#     while (end - start < DT):
#         end = time.time()
#     # ljm.eWriteName(handle,vel_ctrl_pin,0)

#     if (keyboard.is_pressed("q")):
#         # Close connection to LabJack T7
#         u = 0
#         controlAction = send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)

#         # ========== Sensor Data Display ========== #
#         print("Interupted! Current state log:")
#         print(f"Vel_M (rad/s): {motorEncoder.angular_vel:8.2f} |"
#             f"Theta_M (deg): {motorEncoder.angular_pos:8.2f} |"
#             f"Control: {controlAction:5.3f}")
        
#         ljm.close(handle)
#         time.sleep(1)

# Initial setup

last = time.time()
start_program = time.time()
errorPrev = 0
uPrev = 0

while (time.time() - start_program < 1):
    send_control_actions(handle, 0, MOTORPWM_PIN, MOTORDIR_PIN)

def update(frame):
    global count, start_program, uPrev, errorPrev, last
    start = time.time()
    _, motorEncoder.angular_vel = read_encoder(handle, DT, motorEncoder)
    motorEncoder.angular_pos += (motorEncoder.angular_vel * DT / PI) * 180

    current_time = time.time() - start_program

    angleDesired = 360 * np.sin(3 * current_time)
    # angleDesired = 360

    # Control Action Calculation
    error = angleDesired - motorEncoder.angular_pos
    Kp = 0.050
    Kd = 0.005
    Ki = 0.01

    u = Kp * error
    # u = uPrev + (Kp + Ki * DT) * error - Kp * errorPrev
    # u = 0
    uPrev = u
    errorPrev = error

    # if abs(u) < 0.1:
    #     u = 0

    # u = 0

    controlAction = send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)

    # Print data
    print(f"{current_time:.5f} | "
        f"Desired (deg): {angleDesired:8.2f} | "
        f"Vel_M (rad/s): {motorEncoder.angular_vel:8.2f} | "
        f"Theta_M (deg): {motorEncoder.angular_pos:8.2f} | "
        f"Error: {error:8.2f} | "
        f"Error Prev: {errorPrev:8.2f} | "
        f"Control in: {u:8.3f} | "
        f"Control out: {controlAction:8.3f}")
    
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

    end = time.time()
    # while (end - start < DT):
    #     end = time.time()

    if keyboard.is_pressed("q"):
        u = 0
        send_control_actions(handle, u, MOTORPWM_PIN, MOTORDIR_PIN)
        ljm.close(handle)
        print("Interupted! Current state log:")
        print(f"Vel_M (rad/s): {motorEncoder.angular_vel:8.2f} |"
              f"Theta_M (deg): {motorEncoder.angular_pos:8.2f} |"
              f"Control: {controlAction:5.3f}")
        plt.close()
        time.sleep(1)
        exit()

    return line, lineDesired



ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 200), init_func=init, blit=True, interval=DT*1000)
plt.grid(which = "major", linewidth = 1)
plt.grid(which = "minor", linewidth = 0.2)
plt.legend(loc='upper left')
plt.minorticks_on()
plt.show()


