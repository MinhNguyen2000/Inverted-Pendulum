// Pin definitions
#define PENDULUM_ENCODER_A 2
#define PENDULUM_ENCODER_B 3

// Constants definitions

// Variable definitions
int pendEncoderA_stateLast, pendEncoderA_state, pendEncoderB_state;
int counter = 0;
int lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
int oldState, newState;

void setup() {
  pinMode(PENDULUM_ENCODER_A, INPUT_PULLUP);
  pinMode(PENDULUM_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPintoInterrupt(PENDULUM_ENCODER_A), void (*userFunc)(void), int mode)

  Serial.begin(115200);
  pendEncoderA_stateLast = digitalRead(PENDULUM_ENCODER_A);
}

void loop() {
  // Serial.print(digitalRead(PENDULUM_ENCODER_A)); Serial.print(" | ");
  // Serial.println(digitalRead(PENDULUM_ENCODER_B));

  // =============== READ THE SENSOR DATA =============== //
  pendEncoderA_state = digitalRead(PENDULUM_ENCODER_A); // Reads the "current" state of the outputA
  pendEncoderB_state = digitalRead(PENDULUM_ENCODER_B); // Reads the "current" state of the outputB
  newState = pendEncoderB_state * 2 + pendEncoderA_state; // State ranging from 0 to 3 for the quadrature encoderr reading

  // =============== PROCESS THE SENSOR DATA =============== //
  // // If the previous and the current state of the outputA are different, that means a Pulse has occured
  // if (pendEncoderA_state != pendEncoderA_stateLast){     
  //   // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
  //   if (pendEncoderB_state != pendEncoderA_state) { 
  //     counter = counter + 1;
  //   } else {
  //     counter = counter - 1;
  //   }
  //   Serial.print("Position:");
  //   Serial.println(counter);
  // } 
  // pendEncoderA_stateLast = pendEncoderA_state; // Updates the previous state of the outputA with the current state
  counter = counter + lookup_table[oldState * 4 + newState];
  

  // =============== DATA DISPLAY =============== //
  // Serial.print("Position:");
  Serial.println(counter);
  // Serial.print(oldState); Serial.print(" | "); Serial.println(newState);

  oldState = newState;

}
