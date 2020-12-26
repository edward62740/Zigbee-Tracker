HardwareSerial Serial2(PA3, PA2);

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200);
  pinMode(PB11, OUTPUT);
  digitalWrite(PB11, LOW); // zigbee command mode - 0: HEX, 1: AT
}

void loop() {
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
}
