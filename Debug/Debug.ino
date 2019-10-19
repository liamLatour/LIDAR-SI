#include <Wire.h>

float lastValue = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8);
  Wire.onReceive(received);
}

void loop() {
  if (Serial.available() > 0) {
    if (Serial.read() == 1) {
      Serial.println(String(lastValue) + ":0");
    }
  }
}

void received() {
  uint8_t start = Wire.read();
  uint8_t ends = Wire.read();
  lastValue = ((start << 8) + ends) / 100.00;
}
