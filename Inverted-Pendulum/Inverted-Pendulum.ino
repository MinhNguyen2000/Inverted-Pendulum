// Pin definitions
volatile int pendEncoderA_pin = 2;
volatile int pendEncoderB_pin = 3;

// Constants definitions

// Variable definitions
volatile int pendEncoderA_stateLast, pendEncoderA_state, pendEncoderB_state;
volatile long counter = 0;
int lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
int oldState, newState;
volatile bool readState = false;

void encoderInterrupt() {
  readState = true;
}

void setup() {
  pinMode(pendEncoderA_pin, INPUT_PULLUP);
  pinMode(pendEncoderB_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pendEncoderA_pin), encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pendEncoderB_pin), encoderInterrupt, CHANGE);

  Serial.begin(115200);
}

void loop() {
  // Serial.print(digitalRead(PENDULUM_ENCODER_A)); Serial.print(" | ");
  // Serial.println(digitalRead(PENDULUM_ENCODER_B));

  // =============== READ THE SENSOR DATA =============== //
  if (readState) {
    reading = 
    newState = pendEncoderB_state * 2 + pendEncoderA_state; // State ranging from 0 to 3 for the quadrature encoderr reading

    readState = false;
  }
  // =============== PROCESS THE SENSOR DATA =============== //
  counter = counter + lookup_table[oldState * 4 + newState];
  

  // =============== DATA DISPLAY =============== //
  // Serial.print("Position:");
  Serial.println(counter);
  // Serial.print(oldState); Serial.print(" | "); Serial.println(newState);

  oldState = newState;

}
