void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);   // HC-05 on Mega Serial1
  Serial.println("Mega ready");
  delay(2000);
}

void loop() {
  Serial.println("Sending K");
  Serial2.write('K');

  unsigned long startTime = millis();
  bool gotReply = false;

  while (millis() - startTime < 8000) {
    if (Serial2.available()) {
      char reply = Serial2.read();
      Serial.print("Reply from Nano: ");
      Serial.println(reply);

      if (reply == 'O') {
        Serial.println("Feed dispensing complete");
        gotReply = true;
        break;
      }
    }
  }

  if (!gotReply) {
    Serial.println("No reply received");
  }

  delay(5000);
}