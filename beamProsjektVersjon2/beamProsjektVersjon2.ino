#include <Servo.h>

Servo servo1;     //Create servo object to control a servo

const int SERVO_PIN = 9;
const int SENSOR_PIN = A0;

//Global variables necessary for regulator
int setPoint = 25;        //Set point for regulator to work towards
const float Kp = 3.478;    //Proportional part for the regulator
const float Kd = 24.786;    //Differential part for the regulator
const float Ki = 0;    //Integral part for the controller
float error = 0;          //Current error
float oldError = 0;       //Last error
float diffError = 0;      //Difference between current and last error
float intError = 0;       //Accumulating the errors

//Loop variables
unsigned long regulatorTimer = 0;   //Regulator timer
int loopTime = 100;                 //Loop timer

//Variable for storing distances
float distanceArray[10];
int currentIndex = 0;

void setup() {
  servo1.attach(SERVO_PIN);                 // Attaches servo to pin 9 to the servo-object.
  Serial.begin(2000000);                    // Starts the serial monitor
  writeServoPosition(73);                   // Sets the servo to zero degrees as a starting point
  regulatorTimer = millis() + loopTime;     // Sets the first timer for the regulator-loop
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
  if(millis() > regulatorTimer) {
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
    writeServoPosition(positionChange);                               //Sends position to function to change position
    regulatorTimer = millis() + loopTime;                             //Starts new timer
  }
}

/**
 * Changes the position of the servo
 */
void writeServoPosition(float servoPosition) {
  servoPosition = servoPosition + 73; //Added zero-point to the change of position
//  Serial.print("Servoposition: ");
//  Serial.println(servoPosition);
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
  if(5 > error && -5 < error) {
    intError = intError + error;
  }
  if(1 > diffError && -1 < diffError) {
    diffError = 0;
  } else {
    diffError = diffError;
  }
//  Serial.print("Error: ");
//  Serial.println(error);
//  Serial.print(" Diff-error: ");
//  Serial.println(diffError);
//    Serial.print("Int. error: ");
//    Serial.println(intError);
  float u = (Kp * error) + (Kd * diffError) + (Ki * intError);
  return u;
}


/**
 * Convert raw value to distance in centimeters
 */
float convertDistanceValue(long inputDistance) {
  float sensorCm = 0;
  unsigned long x = inputDistance;

  if(205 <= x) {
    sensorCm = (-6.8890*pow(10,-7)*x*x*x) + (8.8527*pow(10,-4)*x*x) + (-0.3996*x) + 65.34;
  } else if (205 > x) {
    sensorCm = (-2.6659*pow(10,-5)*x*x*x) + (0.019*x*x) + (-4.5243*x) + 372.05;
  } else {
    sensorCm = 0;
  }

  //Print all information for debug purposes
//  Serial.print("Real Value: ");
//  Serial.print(x);
//  Serial.print(" Converted: ");
//  Serial.println(sensorCm);
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
