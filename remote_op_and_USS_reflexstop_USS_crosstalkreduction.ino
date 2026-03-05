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
#define FWD_TIME   200
#define REV_TIME   200
#define ROT_TIME   100
#define TURN_TIME  100          // A/D turn time (0.5s)

// ================= ULTRASONIC (HC-SR04) =================
#define OBSTACLE_CM 15

// Sensor 1 = RIGHT (30 deg)
const int TRIG_RIGHT = 12;
const int ECHO_RIGHT = 51;

// Sensor 2 = CENTER
const int TRIG_CENTER = 11;
const int ECHO_CENTER = 53;

// Sensor 3 = LEFT (30 deg)
const int TRIG_LEFT = 13;
const int ECHO_LEFT = 49;

// Crosstalk reduction timing
#define SONAR_GAP_MS 30          // wait between pings (25-30ms recommended)
unsigned long lastSonarPingMs = 0;

// Latest distances (cm)
float distRight  = 999;
float distCenter = 999;
float distLeft   = 999;

// Block flags
bool blockForward   = false;
bool blockTurnLeft  = false;
bool blockTurnRight = false;
bool blockRotate    = false;

// Sonar scan state
enum SonarWhich { SONAR_RIGHT=0, SONAR_CENTER=1, SONAR_LEFT=2 };
SonarWhich sonarState = SONAR_RIGHT;

// ================= STATE =================
String rxBuffer = "";
unsigned long motionStartTime = 0;
unsigned long lastCommandTime = 0;

bool isMoving = false;
char currentMode = 'S';

// ================= FUNCTION DECLS =================
float readHCSR04_cm(int trigPin, int echoPin);
void updateObstacleFlags();              // uses distRight/Center/Left
void sonarStep();                        // reads ONE sensor per call

bool isCmdBlocked(char cmd);
void stopBecauseObstacle(char cmd);

void processPacket(String pkt);

void moveForward();
void moveReverse();
void rotateInPlace();
void turnCCW();
void turnCW();
void stopMotors();

void sendACK(String mode);
void sendERR(String reason);

// ================= SETUP =================
void setup() {
  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE);

  pinMode(TRIG_RIGHT, OUTPUT);  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_CENTER, OUTPUT); pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);   pinMode(ECHO_LEFT, INPUT);

  digitalWrite(TRIG_RIGHT, LOW);
  digitalWrite(TRIG_CENTER, LOW);
  digitalWrite(TRIG_LEFT, LOW);

  stopMotors();
  Serial.println("Mega Ready (Sequential Sonar Reflex ON)");
}

// ================= LOOP =================
void loop() {

  // ---- Sequential sonar scan (one sensor per step) ----
  sonarStep();   // updates distRight/distCenter/distLeft gradually
  updateObstacleFlags();

  // If currently moving in an unsafe mode and it becomes blocked -> stop immediately
  if (isMoving && currentMode != 'R' && isCmdBlocked(currentMode)) {
    stopBecauseObstacle(currentMode);
    sendERR("OBST");
  }

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
        (currentMode == 'C' && elapsed >= ROT_TIME) ||
        (currentMode == 'A' && elapsed >= TURN_TIME) ||
        (currentMode == 'D' && elapsed >= TURN_TIME)) {

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

// ================= SEQUENTIAL SONAR =================
void sonarStep() {
  // Only ping next sensor if enough quiet time has passed
  if (millis() - lastSonarPingMs < SONAR_GAP_MS)
    return;

  lastSonarPingMs = millis();

  switch (sonarState) {
    case SONAR_RIGHT:
      distRight = readHCSR04_cm(TRIG_RIGHT, ECHO_RIGHT);
      sonarState = SONAR_CENTER;
      break;

    case SONAR_CENTER:
      distCenter = readHCSR04_cm(TRIG_CENTER, ECHO_CENTER);
      sonarState = SONAR_LEFT;
      break;

    case SONAR_LEFT:
      distLeft = readHCSR04_cm(TRIG_LEFT, ECHO_LEFT);
      sonarState = SONAR_RIGHT;
      break;
  }

  // Optional debug (uncomment if needed)
  // Serial.print("L/C/R = ");
  // Serial.print(distLeft); Serial.print(" / ");
  // Serial.print(distCenter); Serial.print(" / ");
  // Serial.println(distRight);
}

// Update block flags based on latest distances
void updateObstacleFlags() {
  bool obsR = (distRight  > 0 && distRight  < OBSTACLE_CM);
  bool obsC = (distCenter > 0 && distCenter < OBSTACLE_CM);
  bool obsL = (distLeft   > 0 && distLeft   < OBSTACLE_CM);

  // Requested behavior:
  blockForward   = obsC || obsL || obsR;          // forward blocked by all three sensors
  blockTurnLeft  = obsL || obsC;  // left turn blocked by left or center
  blockTurnRight = obsR || obsC;  // right turn blocked by right or center
  blockRotate    = obsL || obsC || obsR;  // rotate blocked by any
}

bool isCmdBlocked(char cmd) {
  cmd = toupper(cmd);

  if (cmd == 'F') return blockForward;

  // NOTE: Match these to your command meanings:
  // A = turn CW (right)
  // D = turn CCW (left)
  if (cmd == 'D') return blockTurnLeft;
  if (cmd == 'A') return blockTurnRight;

  if (cmd == 'C') return blockRotate;

  return false;
}

void stopBecauseObstacle(char cmd) {
  stopMotors();
  isMoving = false;
  currentMode = 'S';

  Serial.print("[OBST] Blocked cmd ");
  Serial.print(cmd);
  Serial.print(" | L/C/R = ");
  Serial.print(distLeft);
  Serial.print("/");
  Serial.print(distCenter);
  Serial.print("/");
  Serial.println(distRight);
}

// HC-SR04 read (cm). Returns 999 if no echo / out of range.
float readHCSR04_cm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // timeout ~25ms (~4m max)
  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return 999;

  return duration / 58.2f;
}

// ================= PACKET HANDLER =================
void processPacket(String pkt) {

  if (pkt.length() < 3) return;
  if (pkt[0] != '<' || pkt[pkt.length()-1] != '>') return;

  // For your packets: "<CMD:X>"
  // cmd is at index 1 in your earlier version, but if it's "<CMD:F>" then cmd is index 5.
  // We'll support BOTH styles safely.

  char cmd = 0;

  // Style 1: "<F>"
  if (pkt.length() == 3 && pkt[0] == '<' && pkt[2] == '>') {
    cmd = pkt[1];
  }
  // Style 2: "<CMD:F>"
  else if (pkt.startsWith("<CMD:") && pkt.endsWith(">") && pkt.length() >= 7) {
    cmd = pkt[5];
  } else {
    return;
  }

  cmd = toupper(cmd);
  lastCommandTime = millis();

  // Always allow STOP
  if (cmd == 'S') {
    stopMotors();
    isMoving = false;
    currentMode = 'S';
    sendACK("S");
    return;
  }

  // Always allow REVERSE (safe escape)
  if (cmd == 'R') {
    moveReverse();
    motionStartTime = millis();
    currentMode = 'R';
    isMoving = true;
    sendACK("R");
    return;
  }

  // Block unsafe moves when obstacle is close
  if (isCmdBlocked(cmd)) {
    stopBecauseObstacle(cmd);
    sendERR("OBST");
    return;
  }

  // Execute allowed commands
  switch (cmd) {

    case 'F':
      moveForward();
      motionStartTime = millis();
      currentMode = 'F';
      isMoving = true;
      sendACK("F");
      break;

    case 'C':
      rotateInPlace();
      motionStartTime = millis();
      currentMode = 'C';
      isMoving = true;
      sendACK("C");
      break;

    case 'A':  // Turn CW (right) for 0.5s
      turnCW();
      motionStartTime = millis();
      currentMode = 'A';
      isMoving = true;
      sendACK("A");
      break;

    case 'D':  // Turn CCW (left) for 0.5s
      turnCCW();
      motionStartTime = millis();
      currentMode = 'D';
      isMoving = true;
      sendACK("D");
      break;

    default:
      stopMotors();
      isMoving = false;
      currentMode = 'S';
      sendERR("BAD");
      break;
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

void rotateInPlace() {
  motor1.setSpeed(+SPEED_TURN);
  motor2.setSpeed(+SPEED_TURN);   // per your convention
  Serial.println("Rotate");
}

void turnCCW() {
  // If your robot turns the wrong way, swap signs here.
  motor1.setSpeed(-SPEED_TURN);
  motor2.setSpeed(-SPEED_TURN);
  Serial.println("Turn CCW");
}

void turnCW() {
  motor1.setSpeed(+SPEED_TURN);
  motor2.setSpeed(+SPEED_TURN);
  Serial.println("Turn CW");
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  Serial.println("Stop");
}

// ================= ACK/ERR =================
void sendACK(String mode) {
  Serial1.print("<ACK:");
  Serial1.print(mode);
  Serial1.println(">");
}

void sendERR(String reason) {
  Serial1.print("<ERR:");
  Serial1.print(reason);
  Serial1.println(">");
}