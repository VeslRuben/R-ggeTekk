#include <Servo.h>

Servo servo1;                     // create servo object to control a servo

const int SERVO_PIN = 3;
const int SENSOR_PIN = A0;
const int SENSOR_MAX = 600;
const int SENSOR_MIN = 140;

const int P_LEDD = 50;

int setPoint = 25;
float actualPosition = 0;
float positionChange = 0;

float error = 0;
float oldError = 0;
float diffError = 0;

const int Kp = 5;
const int Kd = 10;

boolean debug = false;

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
      actualPosition = readBallPosition();
      positionChange = regulatePosition(actualPosition, setPoint);
      //overføre verdiene til utgangsverdi før di blir skrever til sevoen
      float servoPosition = (positionChange * -1) + 73; // 73 er 0-punktet til servoen
      writeServoPosition(positionChange);
    } else {
      actualPosition = readBallPosition();
    }
  }

void writeServoPosition(float servoPosition) {
  servoPosition = constrain(servoPosition, 6, 140);
  servo1.write(servoPosition);
}

long readBallPosition() {
  long sensorCm = 0;
  long x = 0;
  x = getMeanPos();
  if(253 <= x) {
    sensorCm = ((-3*pow(10,-7))*x*x*x) + (0.0004*x*x) + (-0.2531*x) + 55.596;
  } else if ( x < 253) {
    sensorCm = (0.002*x*x) + (-0.9863*x) + 141.08;
  } else {
    sensorCm = 0;
  }
  Serial.print("Real: ");
  Serial.print(x);
  Serial.print(" Jalla: ");
  Serial.println(sensorCm);
  return sensorCm;
}

float regulatePosition(float actualPos, int wantedPos) {
  oldError = error;
  error = wantedPos - actualPos;
  Serial.print("Error: ");
  Serial.println(error);
  diffError =  error - oldError;
  Serial.print("Diff error: ");
  Serial.println(diffError);
  float u = (Kp * error) + (Kd * diffError);
  return u;
}

long getMeanPos() {
  long tempActualPosition = 0;
  for(int i = 0; i < 10; i++) {
    tempActualPosition = tempActualPosition + analogRead(SENSOR_PIN);
    delay(10);
  }
  return tempActualPosition = tempActualPosition / 10;
}
