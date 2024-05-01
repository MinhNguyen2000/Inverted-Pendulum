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
    FIO6: IN1 to control direction of DC motor
    FIO7: IN2 to control direction of DC motor

"""

import labjack.ljm as ljm
import time
import math

# Constant definition
pi = math.pi

# Variable definition
dt = 0.01

# Pin definition

pendEncoderPin = "DIO0"
pendEncoderPinB = "DIO1"
motorEncoderPin = "DIO2"
motorEncoderPinB = "DIO3"
motorPWMPin = "DIO4"
motorDirPin = "DIO5"
# motorIN1Pin = "DIO6"
# motorIN2Pin = "DIO7"


# Open connection to LabJack T7
handle = ljm.openS("T7", "USB", "ANY")

# =============================================================================
# LabJack Pin and Register Configuration
# =============================================================================

# Configure encoder pins
ljm.eWriteName(handle, pendEncoderPin + "_EF_ENABLE", 0)
ljm.eWriteName(handle, pendEncoderPinB + "_EF_ENABLE", 0)
ljm.eWriteName(handle, pendEncoderPin + "_EF_INDEX", 10)
ljm.eWriteName(handle, pendEncoderPinB + "_EF_INDEX", 10)
ljm.eWriteName(handle, pendEncoderPin + "_EF_ENABLE", 1)
ljm.eWriteName(handle, pendEncoderPinB + "_EF_ENABLE", 1)

ljm.eWriteName(handle, motorEncoderPin + "_EF_ENABLE", 0)
ljm.eWriteName(handle, motorEncoderPinB + "_EF_ENABLE", 0)
ljm.eWriteName(handle, motorEncoderPin + "_EF_INDEX", 10)
ljm.eWriteName(handle, motorEncoderPinB + "_EF_INDEX", 10)
ljm.eWriteName(handle, motorEncoderPin + "_EF_ENABLE", 1)
ljm.eWriteName(handle, motorEncoderPinB + "_EF_ENABLE", 1)


# Configure DC Motor direction pins (IN1 and IN2)
# ljm.eWriteName(handle, motorIN1Pin, 0)                                          # Set default spin direction as brake low
# ljm.eWriteName(handle, motorIN2Pin, 0) 
ljm.eWriteName(handle,motorDirPin,0)

# Configure clock source for PWM output (DC Motor Speed Control)
rollValue= 80000
ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 0)       # Disable the clock source
ljm.eWriteName(handle, "DIO_EF_CLOCK0_DIVISOR", 1)
ljm.eWriteName(handle, "DIO_EF_CLOCK0_ROLL_VALUE", rollValue)
ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 1)

# Configure EF Channel Registers 
PWMDutyCycle = 0                                                                # Default to outputing 0% duty cycle
ljm.eWriteName(handle, motorPWMPin + "_EF_ENABLE", 0);                          # Disable the EF system for initial configuration
ljm.eWriteName(handle, motorPWMPin + "_EF_INDEX", 0);                           # Configure EF system for PWM
ljm.eWriteName(handle, motorPWMPin + "_EF_OPTIONS", 0);                         # Configure what clock source to use: Clock0
ljm.eWriteName(handle, motorPWMPin + "_EF_CONFIG_A", PWMDutyCycle*rollValue);   # Configure duty cycle to be: 50%
ljm.eWriteName(handle, motorPWMPin + "_EF_ENABLE", 1);                          # Enable the EF system, PWM wave is now being outputted


# Define encoder objects blueprint
class Encoder:
    def __init__(self, resolution, angularPos, angularVel, encoderPin):
        self.res = resolution # (pulse per revolution - PPR)
        self.angularPos = angularPos # (rad)
        self.angularVel = angularVel # (rad/s)
        self.pin = encoderPin


# Instantiate encoder objects
pendEncoder = Encoder(2000,0,0,pendEncoderPin + "_EF_READ_A_F_AND_RESET")
motorEncoder = Encoder(3000,0,0,motorEncoderPin + "_EF_READ_A_F_AND_RESET")


# =============================================================================
# Function Definition
# =============================================================================
def read_encoder(dt, labjack_handle, encoder_handle):
    # This function determine the angular velocity of encoder in RPM
    
    # dt = system time step.
    

    # Read encoder value
    val = ljm.eReadName(labjack_handle, encoder_handle.pin)/4

    # Calculate speed (assuming encoder produces 1600 velocitys per revolution)
    speed_rpm = val / (dt * encoder_handle.res) * 60            # (RPM)
    speed_rps = val / (dt * encoder_handle.res) * 2 * pi        # (rad/second)
    # speed = val

    # return [speed_rpm, speed_rps]
    return speed_rpm, speed_rps

def send_control_actions(u, velCtrlPin, dirPin, labjack_handle):
    # u = control action.
    # vel_ctrl_pin = pin to send control to plant, probably DAC1
    # dir_pin = control the direction of the plant, probably DAC0
    
    # Limit control action within [-4.5, 4.5]
    u = max(min(u, 4.5), -4.5)
    
    # Determine the required duty cycle of the PWM signal
    PWMDutyCycle = abs(u) / (4.5 - 0)
    
    # Set direction pin based on control action
    if u > 0:
        # ljm.eWriteName(handle, dirPin1, 0)
        # ljm.eWriteName(handle, dirPin2, 1)
        ljm.eWriteName(handle,dirPin,0)
    elif u <= 0:
        # ljm.eWriteName(handle, dirPin1, 1)
        # ljm.eWriteName(handle, dirPin2, 0)
        ljm.eWriteName(handle,dirPin,1)
    
    # Send control action to velocity control pin
    ljm.eWriteName(labjack_handle, velCtrlPin + "_EF_CONFIG_A", PWMDutyCycle*rollValue)



start_program = time.time()
u = 5

# =============================================================================
# Control Loop
# =============================================================================
while True:
    try:
        start = time.time()
        
        # ========== Obtain data from sensors ========== # 
        [_,pendEncoder.angularVel] = read_encoder(dt, handle, pendEncoder)
        [_,motorEncoder.angularVel] = read_encoder(dt, handle, motorEncoder)
        
        # ========== Sensor Data Processing ========== #
        pendEncoder.angularPos += (pendEncoder.angularVel * dt / math.pi) * 180
        motorEncoder.angularPos += (motorEncoder.angularVel * dt / math.pi) * 180
        
        # ========== Actuation Control ========== #
        # print("Current time: {} | Start time: {}".format(time.time(),start_program))1
        currentTime = time.time() - start_program
        u = 1 * math.sin(2*currentTime)
            

        send_control_actions(u, motorPWMPin, motorDirPin, handle)
        
        # ========== Data Display (Sanity Checkpoint) ========== #
        print("Vel_p(rad/s): {:8.2f} | Theta_P (deg): {:8.2f} | Vel_M (rad/s): {:8.2f} | Theta_M (deg): {:8.2f} | Control: {:5.3f}".format(pendEncoder.angularVel,pendEncoder.angularPos,motorEncoder.angularVel,motorEncoder.angularPos,u))

        # ========== Data Display (Visualization) ========== #
        
        
        # Ensure each loop takes the same amount of time as the sampling time
        end = time.time()
        # print("Elapsed time: {:.3e}".format(end-start))
        while (end - start < dt):
            end = time.time()
        # ljm.eWriteName(handle,vel_ctrl_pin,0)
    
    except KeyboardInterrupt:
        # Close connection to LabJack T7
        send_control_actions(0, motorPWMPin, motorDirPin, handle)
        ljm.close(handle)


