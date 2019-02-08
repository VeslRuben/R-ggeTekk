#include <Servo.h>

Servo servo1;                     // create servo object to control a servo

const int SERVO_PIN = 3;
const int SENSOR_PIN = A0;
const int SENSOR_MAX = 600;
const int SENSOR_MIN = 140;

const int P_LEDD = 50;

int setPoint = 0;
float actualPosition = 0;
float positionChange = 0;

boolean debug = true;

void setup() {
  servo1.attach(SERVO_PIN);       // attaches the servo on pin 3 to the servo object
  writeServoPosition(73);         // Servo goes to zero degrees
  Serial.begin(9600);
}

void loop() {
  if(!debug) {
    if(Serial.available()) {
      setPoint = Serial.parseInt();
      Serial.println(setPoint);
      }
      float actualPosition = getPos();
      positionChange = regulatePosition(actualPosition, setPoint);
      //overføre verdiene til utgangsverdi før di blir skrever til sevoen
      servoPosition = positionChange + 73; // 73 er 0punget til servoen
      writeServoPosition(positionChange);
    } else {
      for(int i = 0; i < 10; i++) {
        actualPosition = actualPosition + analogRead(SENSOR_PIN);
        delay(10);
      }
      actualPosition = actualPosition / 10;
      Serial.println(actualPosition);
    }
  }

void writeServoPosition(float servoPosition) {
  servoPosition = constrain(servoPosition, 6, 140);
  servo1.write(servoPosition);
}

float readBallPosition() {
  float sensorCm = 0;
  float sensorRealValue = 0;
  sensorRealValue = analogRead(SENSOR_PIN);
  if (sensorRealValue >= 390) {
      //Lineær fra 0 - 5cm
      sensorCm = (594 - sensorRealValue) / 40.8;
  } else if (280 <= sensorRealValue && sensorRealValue < 390) {
      //Lineær fra 5 - 10cm
      sensorCm = ((390 - sensorRealValue) / 22) + 5;
  } else if (193 <= sensorRealValue && sensorRealValue < 280) {
      //Lineær fra 10 - 20cm
      sensorCm = ((280 - sensorRealValue) / 8.7) + 10;
  } else if (153 <= sensorRealValue && sensorRealValue < 193) {
      //Lineær fra 20 - 30cm
      sensorCm = ((193 - sensorRealValue) / 4) + 20;
  } else if (125 <= sensorRealValue && sensorRealValue < 153) {
      //Lineær fra 30 - 45cm
      sensorCm = ((153 - sensorRealValue) / 2) + 30;
  }
  Serial.print("Real: ");
  Serial.print(sensorRealValue);
  Serial.print(" Jalla: ");
  Serial.println(sensorCm);
  return sensorCm;
}

float regulatePosition(float actualPosition, int setPoint) {
  float error = setPoint - actualPosition;
  int p_constant = 5;
  error = error * p_constant;
  return error;
}

float getPos() {
  float actualPosition;
  for(int i = 0; i < 10; i++) {
    actualPosition = actualPosition + readBallPosition();
  }
  return actualPosition = actualPosition / 10;
}
