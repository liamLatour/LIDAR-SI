#include <Wire.h>

int pinA = 4;
int pinB = 2;
volatile int pos = 0;

int in1 = 9; //Declaring the pins where in1 in2 from the driver are wired
int in2 = 8; //here they are wired with D9 and D8 from Arduino
int ConA = 5; //And we add the pin to control the speed after we remove its jumper
//Make sure it's connected to a pin that can deliver a PWM signal

int dist;
int motorSpeed = 150;

void setup() {
  Serial.begin(2000000);
  Wire.begin(8);
  Wire.onReceive(received);

  pinMode(in1, OUTPUT); //Declaring the pin modes
  pinMode(in2, OUTPUT);
  pinMode(ConA, OUTPUT);

  pinMode(pinB, INPUT);
  pinMode(pinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinB), front, CHANGE);
}

void received() {
  uint8_t start = Wire.read();
  uint8_t ends = Wire.read();
  dist = ((start << 8) + ends);
}

void loop() {
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();    
    switch (incomingByte)
    {
      case 0:
        Serial.println(String(dist) + ":" + String(pos / 180.0 * PI));
        break;
      case 1:
        TurnOnA();
        break;
      case 2:
        TurnOffA();
        break;
      default:
        motorSpeed = incomingByte;
        break;
    }
    Serial.write(motorSpeed);
  }
}

void TurnOnA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ConA, motorSpeed);
}

void TurnOffA() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ConA, 0);
}

void front() {
  if (digitalRead(pinA) == digitalRead(pinB)) {
    ++pos;
  }
  else {
    --pos;
  }
  pos = pos % 360;
}
