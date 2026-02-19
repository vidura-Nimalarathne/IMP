#include "CytronMotorDriver.h"

// ================= MOTORS =================
CytronMD motor1(PWM_DIR, 2, 3);
CytronMD motor2(PWM_DIR, 4, 5);

const int SPEED_FWD  = 128;
const int SPEED_TURN = 140;
const int SPEED_REV  = 128;

// ================= UART =================
#define BAUD_RATE 9600
#define WATCHDOG_TIMEOUT 1500   // ms

// ================= MOTION TIMES =================
#define FWD_TIME 2000
#define REV_TIME 2000
#define ROT_TIME 1000

// ================= STATE =================
String rxBuffer = "";
unsigned long motionStartTime = 0;
unsigned long lastCommandTime = 0;

bool isMoving = false;
char currentMode = 'S';

// ================= SETUP =================
void setup() {
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);

  stopMotors();
  Serial.println("Mega Ready");
}

// ================= LOOP =================
void loop() {

  // ----- UART READ -----
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      processPacket(rxBuffer);
      rxBuffer = "";
    } else {
      rxBuffer += c;
    }
  }

  // ----- Motion timing control -----
  if (isMoving) {
    unsigned long elapsed = millis() - motionStartTime;

    if ((currentMode == 'F' && elapsed >= FWD_TIME) ||
        (currentMode == 'R' && elapsed >= REV_TIME) ||
        (currentMode == 'C' && elapsed >= ROT_TIME)) {

      stopMotors();
      isMoving = false;
      currentMode = 'S';
    }
  }

  // ----- Watchdog -----
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    stopMotors();
    isMoving = false;
    currentMode = 'S';
  }
}

// ================= PACKET HANDLER =================
void processPacket(String pkt) {

  if (pkt.length() < 3) return;
  if (pkt[0] != '<' || pkt[pkt.length()-1] != '>') return;

  char cmd = pkt[1];
  lastCommandTime = millis();

  switch (cmd) {

    case 'F':
      moveForward();
      motionStartTime = millis();
      currentMode = 'F';
      isMoving = true;
      sendACK("F");
      break;

    case 'R':
      moveReverse();
      motionStartTime = millis();
      currentMode = 'R';
      isMoving = true;
      sendACK("R");
      break;

    case 'C':
      rotateInPlace();
      motionStartTime = millis();
      currentMode = 'C';
      isMoving = true;
      sendACK("C");
      break;

    case 'S':
      stopMotors();
      isMoving = false;
      currentMode = 'S';
      sendACK("S");
      break;
  }
}

// ================= MOTOR FUNCTIONS =================

// FORWARD  = motor1 +, motor2 -
void moveForward() {
  motor1.setSpeed(+SPEED_FWD);
  motor2.setSpeed(-SPEED_FWD);
  Serial.println("Forward");
}

// REVERSE = motor1 -, motor2 +
void moveReverse() {
  motor1.setSpeed(-SPEED_REV);
  motor2.setSpeed(+SPEED_REV);
  Serial.println("Reverse");
}

// Rotate in place (spin right)
void rotateInPlace() {
  motor1.setSpeed(+SPEED_TURN);
  motor2.setSpeed(+SPEED_TURN);  // per your convention
  Serial.println("Rotate");
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  Serial.println("Stop");
}

// ================= ACK =================
void sendACK(String mode) {
  Serial1.print("<ACK:");
  Serial1.print(mode);
  Serial1.println(">");
}
