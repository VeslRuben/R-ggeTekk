#include <Servo.h>

Servo servo1;                     // create servo object to control a servo

const int SERVO_PIN = 5;
const int SENSOR_PIN = A5;
const int SENSOR_MAX = 600;
const int SENSOR_MIN = 140;

int setPoint = 25;
float distanceArray[10];
int currentIndex = 0;
unsigned long regulatorTimer = 0;

int loopTime = 100;

float error = 0;
float oldError = 0;
float diffError = 0;

const int Kp = 5;
const int Kd = 10;

boolean debug = false;

void setup() {
  servo1.attach(SERVO_PIN);       // attaches the servo on pin 3 to the servo object
  writeServoPosition(73);         // Servo goes to zero degrees
  Serial.begin(250000);
  regulatorTimer = millis() + loopTime;
  
}

void loop() {
  if(!debug) {
    if(Serial.available()) {
      setPoint = Serial.parseInt();
      Serial.println(setPoint);
      }
      distanceArray[currentIndex] = getMeanPos();
      currentIndex++;
      if(9 < currentIndex) {
        currentIndex = 0;
      }
      if(millis() > regulatorTimer) {
        int tempMax = distanceArray[0];
        int tempMin = distanceArray[0];
        unsigned long meanDistance = 0;
        for(int i = 1; i < 10; i++) {
          if(distanceArray[i] > tempMax) {
            tempMax = distanceArray[i];
          }
          if(distanceArray[i] < tempMin) {
            tempMin = distanceArray[i];
          }
          meanDistance = meanDistance + distanceArray[i];
          
        }
        Serial.println("Test");
        meanDistance = (meanDistance - tempMax - tempMin) / 8;
        float ballPosition = readBallPosition(meanDistance);
        float positionChange = regulatePosition(ballPosition, setPoint);
        float servoPosition = (positionChange * -1) + 73; // 73 er 0-punktet til servoen
        writeServoPosition(positionChange);
        regulatorTimer = millis() + loopTime;
      }
    } else {
      //actualPosition = readBallPosition();
    }
  }

void writeServoPosition(float servoPosition) {
  servoPosition = constrain(servoPosition, 6, 140);
  servo1.write(servoPosition);
}

unsigned long readBallPosition(unsigned long meanDistance) {
  unsigned long sensorCm = 0;
  unsigned long x = meanDistance;
  if(263 <= x) {
    sensorCm = ((9.3482*pow(10,-5))*x*x) + (-0.1175*x) + 38.2;
  } else if (263 > x) {
    sensorCm = (0.0015*x*x) + (-0.8275*x) + 126,75;
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
  int tempActualPosition[10];
  for(int i = 0; i < 10; i++) {
    tempActualPosition[i] = analogRead(SENSOR_PIN);
  }
  int tempIndexMax = tempActualPosition[0];
  int tempIndexMin = tempActualPosition[0];
  for(int i = 1; i < 10; i++) {
    if(tempActualPosition[i] > tempIndexMax) {
      tempIndexMax = i;
    }
    if(tempActualPosition[i] < tempIndexMin) {
      tempIndexMin = i;
    }
  }
  tempActualPosition[tempIndexMax] = 0;
  tempActualPosition[tempIndexMin] = 0;
  float tempMeanPosition = 0;
  for(int i = 0; i < 10; i++) {
    tempMeanPosition = tempMeanPosition + tempActualPosition[i];
  }
  return tempMeanPosition / 8;
}
