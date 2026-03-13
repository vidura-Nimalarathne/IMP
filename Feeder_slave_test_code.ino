#include <SoftwareSerial.h>

SoftwareSerial BT(2,3); // RX,TX

const int motorPin = 11;
const unsigned long dispenseTime = 5000;

bool busy = false;

void setup() {
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW); // motor OFF

  Serial.begin(9600);
  BT.begin(9600);

  Serial.println("Nano feeder ready");
}

void loop() {

  if (BT.available()) {
    char cmd = BT.read();

    if (cmd == 'K' && !busy) {

      busy = true;
      Serial.println("Dispensing feed");

      digitalWrite(motorPin, HIGH);   // motor ON
      delay(dispenseTime);
      digitalWrite(motorPin, LOW);  // motor OFF

      BT.write('O');  // send completion
      busy = false;
    }
  }
}