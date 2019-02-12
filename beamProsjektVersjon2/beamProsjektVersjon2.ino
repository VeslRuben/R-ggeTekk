#include <Servo.h>

Servo servo1;     //Create servo object to control a servo

const int SERVO_PIN = 9;
const int SENSOR_PIN = A5;

//Global variables necessary for regulator
int setPoint = 25;    //Set point for regulator to work towards
const int Kp = 5;     //Proportional part for the regulator
const int Kd = 10;    //Differential part for the regulator
float error = 0;      //Current error
float oldError = 0;   //Last error
float diffError = 0;  //Difference between current and last error

//Loop variables
unsigned long regulatorTimer = 0;   //Regulator timer
int loopTime = 100;                 //Loop timer

//Variable for storing distances
float distanceArray[10];
int currentIndex = 0;

void setup() {
  servo1.attach(SERVO_PIN);                 // Attaches servo to pin 9 to the servo-object.
  Serial.begin(9600);
  writeServoPosition(73);
  regulatorTimer = millis() + loopTime;     //Sets the first timer for the regulator-loop
}

void loop() {
  //Checks for input for changing set point
  if(Serial.available()) {
    setPoint = Serial.parseInt();
    Serial.print("New Set Point is: ");
    Serial.println(setPoint);
  }
  
  //Build up array of distances to build an average value
  distanceArray[currentIndex] = getMeanPos();
  currentIndex++;
  if(9 < currentIndex) {
    currentIndex = 0;
  }
  
  //Check if timer for regulating position is ready
  if(millis() < regulatorTimer) {
    int tempMaxValue = distanceArray[0];
    int tempMinValue = distanceArray[0];
    unsigned long meanDistance = 0;
    for(int i = 0; i < 10; i++) {
      if(tempMaxValue < distanceArray[i]) {
        tempMaxValue = distanceArray[i];
      }
      if(tempMinValue > distanceArray[i]) {
        tempMinValue = distanceArray[i];
      }
      meanDistance = meanDistance + distanceArray[i];
    }
    meanDistance = (meanDistance - tempMaxValue - tempMinValue) / 8;  //Builds average of averages

    //Set up values for how much to change the servo-position
    float ballPosition = convertDistanceValue(meanDistance);
    float positionChange = positionRegulator(ballPosition, setPoint);
    float servoPosition = (positionChange * -1) + 73;                 //Change polarity of position change (for getting right direction), add 73 for zero-point
    writeServoPosition(positionChange);                               //Sends position to function to change position
    regulatorTimer = millis() + loopTime;                             //Starts new timer
  }
}

/**
 * Changes the position of the servo
 */
void writeServoPosition(float servoPosition) {
  servoPosition = constrain(servoPosition, 6, 140);
  servo1.write(servoPosition);
}

/**
 * Takes in current- and wanted position.
 * Calculates error, diff-error and calculates
 * how much to change position.
 */
float positionRegulator(float actualPos, int wantedPos) {
  oldError = error;
  error = wantedPos - actualPos;
  diffError = error - oldError;
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" Diff-error: ");
  Serial.println(diffError);
  float u = (Kp * error) + (Kd * diffError);
  return u;
}


/**
 * Convert raw value to distance in centimeters
 */
long convertDistanceValue(long inputDistance) {
  unsigned long sensorCm = 0;
  unsigned long x = inputDistance;

  if(263 <= x) {
    sensorCm = ((9.3482*pow(10,-5))*x*x) + (-0.1175*x) + 38.2;
  } else if (263 > x) {
    sensorCm = (0.0015*x*x) + (-0.8275*x) + 126.75;
  } else {
    sensorCm = 0;
  }

  //Print all information for debug purposes
  Serial.print("Real Value: ");
  Serial.print(x);
  Serial.print(" Converted: ");
  Serial.println(sensorCm);
  return sensorCm;
}

/**
 * Takes in 10 values, removes highest and lowest value.
 * Builds an average.
 */
long getMeanPos() {
  int tempActualPosition[10];
  for(int i = 0; i < 10; i++) {
    tempActualPosition[i] = analogRead(SENSOR_PIN);
  }
  int tempMaxValue = tempActualPosition[0];
  int tempMinValue = tempActualPosition[0];
  float tempMeanPosition = 0;
  for(int i = 0; i < 10; i++) {
    if(tempMaxValue < tempActualPosition[i]) {
      tempMaxValue = tempActualPosition[i];
    }
    if(tempMinValue > tempActualPosition[i]) {
      tempMinValue = tempActualPosition[i];
    }
    tempMeanPosition = tempMeanPosition + tempActualPosition[i];
  }
  return (tempMeanPosition - tempMinValue - tempMaxValue) / 8;
}
