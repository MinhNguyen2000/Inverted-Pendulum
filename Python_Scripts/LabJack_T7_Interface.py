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
import keyboard                             # To obtain user input (key presses)

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
DT = 0.0025                         # Sampling time

# State definition
current_state = "startup"


# =============================================================================
# LabJack Initialization and Pin Configuration Functions
# =============================================================================
def initialize_labjack():
    handle = ljm.openS("T7", "USB", "ANY")
    print("LabJack T7 connected")
    return handle

class Encoder:
    def __init__(self, name, resolution, encoder_pin, encoder_pinb, angular_pos = 0, angular_vel = 0):
        self.name = name
        self.res = resolution               # (pulse per revolution - PPR)
        self.encoder_pin = encoder_pin              # the quadrature encoder read pin
        self.encoder_pinb = encoder_pinb
        self.angular_pos = angular_pos      # angular position (rad) - initialized as 0
        self.angular_vel = angular_vel      # angular velovity (rad/s) - initialized as 0

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
    
    def send_control_actions(self, labjack_handle, ctrl_action):
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

        # Set direction pin based on control action
        if ctrl_action > 0:
            ljm.eWriteName(labjack_handle,self.motordir_pin,0)
        elif ctrl_action <= 0:
            ljm.eWriteName(labjack_handle,self.motordir_pin,1)
        
        # Send control action to velocity control pin
        ljm.eWriteName(labjack_handle, self.motorpwm_pin + "_EF_CONFIG_A", pwm_duty_cycle*ROLL_VALUE)

        return ctrl_action


# =============================================================================
# Low-level Sensor and Actuator Control Functions
# =============================================================================
def read_limitswitch(labjack_handle, limitswitch_pin):
    '''
    Function for reading the digital output of the limit switches
    '''
    val = ljm.eReadName(labjack_handle,limitswitch_pin)
    return val


# =============================================================================
# Main Program
# =============================================================================

def main():
    global current_state
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
    motor.intialize_motor()

    
    # Main control loop
    try:
        while True:
            [_,pendEncoder.angular_vel] = pendEncoder.read_encoder(handle, dt=DT)
            [_,motorEncoder.angular_vel] = motorEncoder.read_encoder(handle, dt=DT)

            # ========== Sensor Data Processing ========== # 
            pendEncoder.angular_pos += (pendEncoder.angular_vel * DT / PI) * 180
            motorEncoder.angular_pos += (motorEncoder.angular_vel * DT / PI) * 180

            cart_position = motorEncoder.angular_pos / 360 * 0.205

            # ========== Sensor Data Display ========== #
            print(f"Vel_p (rad/s): {pendEncoder.angular_vel:6.1f} |"
                f"Theta_P (deg): {pendEncoder.angular_pos:6.1f} |"
                f"Vel_M (rad/s): {motorEncoder.angular_vel:6.1f} |"
                f"Theta_M (deg): {motorEncoder.angular_pos:6.1f} |"
                f"Distance (m): {cart_position:5.2f} | "
            )
            
    except KeyboardInterrupt:
        pass

    finally: 
        pass

if __name__ == "__main__":
    main()