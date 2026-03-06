#include "CytronMotorDriver.h"

// ================= MOTORS =================
CytronMD motor1(PWM_DIR, 2, 3);
CytronMD motor2(PWM_DIR, 4, 5);

const int SPEED_FWD = 128;
const int SPEED_REV = 128;

// ================= ENCODERS =================
// Your current wiring
const int LEFT_ENC_A  = 38;
const int LEFT_ENC_B  = 39;
const int RIGHT_ENC_A = 43;
const int RIGHT_ENC_B = 42;

// Encoder counts
volatile long leftCount = 0;
volatile long rightCount = 0;

// Previous AB states
uint8_t prevLeftState = 0;
uint8_t prevRightState = 0;

// Quadrature lookup table
// index = (prevState << 2) | currState
const int8_t qem[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ================= TEST STATE MACHINE =================
enum TestState {
  MOVE_FORWARD,
  STOP_AFTER_FORWARD,
  MOVE_REVERSE,
  STOP_AFTER_REVERSE
};

TestState state = MOVE_FORWARD;
unsigned long stateStartTime = 0;
unsigned long lastPrintTime = 0;

// ================= HELPERS =================
uint8_t readEncoderState(int pinA, int pinB) {
  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  return (a << 1) | b;
}

void updateEncoders() {
  // ----- Left encoder -----
  uint8_t currLeftState = readEncoderState(LEFT_ENC_A, LEFT_ENC_B);
  if (currLeftState != prevLeftState) {
    uint8_t index = (prevLeftState << 2) | currLeftState;
    leftCount += qem[index];
    prevLeftState = currLeftState;
  }

  // ----- Right encoder -----
  uint8_t currRightState = readEncoderState(RIGHT_ENC_A, RIGHT_ENC_B);
  if (currRightState != prevRightState) {
    uint8_t index = (prevRightState << 2) | currRightState;
    rightCount += qem[index];
    prevRightState = currRightState;
  }
}

// ================= MOTOR FUNCTIONS =================
void moveForward() {
  motor1.setSpeed(+SPEED_FWD);
  motor2.setSpeed(-SPEED_FWD);
  Serial.println("Forward");
}

void moveReverse() {
  motor1.setSpeed(-SPEED_REV);
  motor2.setSpeed(+SPEED_REV);
  Serial.println("Reverse");
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  Serial.println("Stop");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Read initial encoder states
  prevLeftState = readEncoderState(LEFT_ENC_A, LEFT_ENC_B);
  prevRightState = readEncoderState(RIGHT_ENC_A, RIGHT_ENC_B);

  stopMotors();

  Serial.println("=== Encoder Motor Test Start ===");
  Serial.println("Sequence: Forward 3s -> Stop 1s -> Reverse 3s -> Stop 1s");
  Serial.println("Printing encoder counts every 200 ms");

  state = MOVE_FORWARD;
  stateStartTime = millis();
  moveForward();
}

// ================= LOOP =================
void loop() {
  // Continuously poll encoders
  updateEncoders();

  unsigned long now = millis();

  // Print counts every 200 ms
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;

    Serial.print("Left Count: ");
    Serial.print(leftCount);
    Serial.print("    Right Count: ");
    Serial.println(rightCount);
  }

  // State machine
  switch (state) {
    case MOVE_FORWARD:
      if (now - stateStartTime >= 3000) {
        stopMotors();
        state = STOP_AFTER_FORWARD;
        stateStartTime = now;
      }
      break;

    case STOP_AFTER_FORWARD:
      if (now - stateStartTime >= 1000) {
        moveReverse();
        state = MOVE_REVERSE;
        stateStartTime = now;
      }
      break;

    case MOVE_REVERSE:
      if (now - stateStartTime >= 3000) {
        stopMotors();
        state = STOP_AFTER_REVERSE;
        stateStartTime = now;
      }
      break;

    case STOP_AFTER_REVERSE:
      if (now - stateStartTime >= 1000) {
        moveForward();
        state = MOVE_FORWARD;
        stateStartTime = now;
      }
      break;
  }
}