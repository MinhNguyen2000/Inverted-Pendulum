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


# Constant definition
PI = math.pi
SPEED_OF_SOUND = 343

# Variable definition
DT = 0.05

# Pin definition

PENDENCODER_PIN = "DIO0"
PENDENCODER_PINB = "DIO1"
MOTORENCODER_PIN = "DIO2"
MOTORENCODER_PINB = "DIO3"
MOTORPWM_PIN = "DIO4"
MOTORDIR_PIN = "DIO5"

ULTRASONICTRIG_PIN = "DIO6"
ULTRASONICECHO_PIN = "DIO7"
# motorIN1Pin = "DIO6"
# motorIN2Pin = "DIO7"


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
motorEncoder = Encoder(3000,0,0,MOTORENCODER_PIN + "_EF_READ_A_F_AND_RESET")


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
    ctrl_action = max(min(ctrl_action, 4.5), -4.5)
    # Determine the required duty cycle of the PWM signal
    pwm_duty_cycle = abs(ctrl_action) / (4.5 - 0)
    # Set direction pin based on control action
    if ctrl_action > 0:
        ljm.eWriteName(labjack_handle,dir_pin,0)
    elif ctrl_action <= 0:
        ljm.eWriteName(labjack_handle,dir_pin,1)
    
    # Send control action to velocity control pin
    ljm.eWriteName(labjack_handle, vel_ctrl_pin + "_EF_CONFIG_A", pwm_duty_cycle*ROLL_VALUE)
    

def send_trigger_pulse(labjack_handle,trig_pin):
    ljm.eWriteName(labjack_handle, trig_pin, 1)
    start_trig_time = time.time()  # 10 microseconds
    while time.time() - start_trig_time < 0.000001:
        pass
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
start_program = time.time()
send_control_actions(handle, 0, MOTORPWM_PIN, MOTORDIR_PIN)

while True:
    try:
        start = time.time()
        
        # ========== Obtain data from sensors ========== # 
        [_,pendEncoder.angular_vel] = read_encoder(handle, DT, pendEncoder)
        [_,motorEncoder.angular_vel] = read_encoder(handle, DT, motorEncoder)


        # Ensure each loop takes the same amount of time as the sampling time
        end = time.time()
        # print("Elapsed time: {:.3e}".format(end-start))
        while (end - start < DT):
            end = time.time()
        # ljm.eWriteName(handle,vel_ctrl_pin,0)
    
    except KeyboardInterrupt:
        # Close connection to LabJack T7
        send_control_actions(handle, 0, MOTORPWM_PIN, MOTORDIR_PIN)
        ljm.close(handle)


