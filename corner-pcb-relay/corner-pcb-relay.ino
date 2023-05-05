#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;
// Set up a new SoftwareSerial object
SoftwareSerial mySerial (rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(115200);
  Serial.begin(115200);
  Serial.println("Reading");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(mySerial.available()){
    Serial.print((char) mySerial.read());
  }
  delay(20);
}
