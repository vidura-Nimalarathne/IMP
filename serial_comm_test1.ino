void setup() {
  Serial.begin(9600);      // USB debug to laptop
  Serial1.begin(9600);     // UART pins 18/19
  Serial.println("Mega booted");
}

void loop() {
  Serial1.print("hello_jetson");
  Serial1.println(millis());     // send text + number
  Serial.println("Sent on Serial1");
  delay(1000);
}
