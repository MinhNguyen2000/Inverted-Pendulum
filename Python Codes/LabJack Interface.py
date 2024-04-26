import u3
import time
import matplotlib.pyplot as plt

# Initialize LabJack U3-HV
d = u3.U3()

# Define digital input pins
pendEncoderPinA = 4  # Example: FIO0, FIO1, FIO2
pendEncoderPinB = 5
pendEncoderPin = [4, 5]
counter = 0
state_old = 0
state_new = 0

transition_table = [[0, 1, -1, 0],[-1, 0, 0, 1],[1, 0, 0, -1],[0, -1, 1, 0]]

try:
    while True:
        # ========== Read digital input values ========== #
        # pendEncoderA = d.getFeedback(u3.BitStateRead(pendEncoderPinA))[0]
        # pendEncoderB = d.getFeedback(u3.BitStateRead(pendEncoderPinB))[0]
        digital_input = [d.getFeedback(u3.BitStateRead(pin))[0] for pin in pendEncoderPin]
        pendEncoderA = digital_input[0]
        pendEncoderB = digital_input[1]
        
        state_new = pendEncoderB * 2 + pendEncoderA
        counter = counter + transition_table[state_old][state_new]
        
        # ========== Print digital input values ========== #
        print(pendEncoderB,' | ', pendEncoderA, ' | ', state_new, ' | ','{0:5d}'.format(counter))
        # print("Counter value: {0:5d}".format(counter))
        

        state_old = state_new
    

        # Delay for stability
        # time.sleep(0.1)
except KeyboardInterrupt:
    # Clean up
    d.close()  # Close connection to LabJack
