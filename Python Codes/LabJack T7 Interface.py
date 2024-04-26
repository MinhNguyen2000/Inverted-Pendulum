# -*- coding: utf-8 -*-
"""
Created on Fri Apr 26 12:55:00 2024

@author: Minh Nguyen
"""

import labjack.ljm as ljm
import time
import math

# Constant definition
pi = math.pi

# Pin definition
dir_pin = "DAC0"
vel_ctrl_pin = "DAC1"

# Variable definition
dt = 0.01
resolution = 2000 # (pulse per revolution - PPR)
theta = 0

# Open connection to LabJack T7
handle = ljm.openS("T7", "USB", "ANY")

# Configure control pins
ljm.eWriteName(handle, dir_pin, 5)
ljm.eWriteName(handle, vel_ctrl_pin, 0)
   
# Configure encoder pins
ljm.eWriteName(handle, "DIO0_EF_ENABLE", 0)
ljm.eWriteName(handle, "DIO1_EF_ENABLE", 0)
ljm.eWriteName(handle, "DIO0_EF_INDEX", 10)
ljm.eWriteName(handle, "DIO1_EF_INDEX", 10)
ljm.eWriteName(handle, "DIO0_EF_ENABLE", 1)
ljm.eWriteName(handle, "DIO1_EF_ENABLE", 1)

def read_encoder(dt, handle):
    # This function determine the angular velocity of encoder in RPM
    
    # dt = system time step.
    
    pin_val = 'DIO0_EF_READ_A_F_AND_RESET'

    # Read encoder value
    val = ljm.eReadName(handle, pin_val)/4

    # Calculate speed (assuming encoder produces 1600 velocitys per revolution)
    speed_rpm = val / (dt * resolution) * 60            # (RPM)
    speed_rps = val / (dt * resolution) * 2 * pi        # (rad/second)
    speed_dpst = val / resolution * 360                 # (degree/sample time)
    # speed = val

    # return [speed_rpm, speed_rps]
    return speed_rpm, speed_rps, speed_dpst

def send_control_actions(u, vel_ctrl_pin, dir_pin, handle):
    # u = control action.
    # vel_ctrl_pin = pin to send control to plant, probably DAC1
    # dir_pin = control the direction of the plant, probably DAC0
    
    # Limit control action within [-4.5, 4.5]
    u = max(min(u, 4.5), -4.5)
    
    # Set direction pin based on control action
    if u > 0:
        ljm.eWriteName(handle, dir_pin, 0)
    elif u <= 0:
        ljm.eWriteName(handle, dir_pin, 5)
    
    # Send control action to velocity control pin
    ljm.eWriteName(handle, vel_ctrl_pin, abs(u))


while True:
    try:
        start = time.time()
        
        # ========== Obtain data from sensors ========== #
        vel_rpm,vel_rps,vel_dpst = read_encoder(dt, handle)
        
        # ========== Sensor Data Processing ========== #
        theta = theta + ((vel_rps * dt) / pi) * 180
        
        # ========== Data Display (Sasnity Checkpoint) ========== #
        print("Velocity (rpm): {:8.2f} | Theta (deg): {:8.2f}".format(vel_rpm,theta))
        
        # Ensure each loop takes the same amount of time as the sampling time
        end = time.time()
        # print("Elapsed time: {:.3e}".format(end-start))
        while (end - start < dt):
            end = time.time()
    
    except KeyboardInterrupt:
        # Close connection to LabJack T7
        ljm.close(handle)


