#include <Wire.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Variables to adjust
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

// PID variables
double setpoint = 35;  // Setpoint distance from the wall in mm
double kp = 1.0;       // Proportional gain
double ki = 0.0;       // Integral gain
double kd = 0.0;       // Derivative gain

int motorMapSpeed = 255;
int forwardSpeed = 50;
int turnSpeedLeftMotor = 100;
int turnSpeedRightMotor = 100;
int leftMotorSpeedCorrection = 0;
int rightMotorSpeedCorrection = 0;

int moveForwardTime = 1000;
int turnLeftTime = 100;
int turnRightTime = 100;
int normalPIDDelayTime = 0;

int leftWallDetectThreshold = 150;
int rightWallDetectThreshold = 150;
int frontWallDetectThreshold = 150;

int timeDelayLEDPattern = 200;




///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


int errorF = 5;
int errorL = 5;
int errorR = 20;

int readingF;
int readingR;
int readingL;

const int leftMotorA = 32;
const int leftMotorB = 33;
const int rightMotorA = 25;
const int rightMotorB = 26;
const int inbuiltLED = 2;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

double error1, error2;
double integral1 = 0, integral2 = 0;
double lastError1 = 0, lastError2 = 0;

bool leftWallPresent = true;
bool rightWallPresent = true;
bool frontWallPresent = true;


void setup() {
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(inbuiltLED, OUTPUT);

  Wire.setClock(400000);
  Wire.begin();

  TCA9548A(1);
  if (!lox1.begin()) {
    while (1)
      ;
  }
  TCA9548A(2);
  if (!lox2.begin()) {
    while (1)
      ;
  }
  TCA9548A(7);
  if (!lox3.begin()) {
    while (1)
      ;
  }
}


void loop() {
  runMotorByGivenSpeed(2, HIGH);
  // moveForwardOneSquare();
  // detectWallsAround();
  // delay(5000);
}



void runMotorByGivenSpeed(int motorID, int speed) {
  if (motorID == 1) {
    if (speed > 0) {
      analogWrite(leftMotorA, abs(speed + leftMotorSpeedCorrection));
      analogWrite(leftMotorB, LOW);
    } else {
      analogWrite(leftMotorB, abs(speed + leftMotorSpeedCorrection));
      analogWrite(leftMotorA, LOW);
    }
  } else {
    if (speed > 0) {
      analogWrite(rightMotorA, abs(speed + rightMotorSpeedCorrection));
      analogWrite(rightMotorB, LOW);
    } else {
      analogWrite(rightMotorB, abs(speed + rightMotorSpeedCorrection));
      analogWrite(rightMotorA, LOW);
    }
  }
}


void travelWithPID() {
  getTOFValues(lox1, 1, errorF);
  getTOFValues(lox2, 2, errorR);
  getTOFValues(lox3, 7, errorL);

  int distanceLeft = readingL;
  int distanceRight = readingR;

  // Calculate PID errors
  error1 = setpoint - distanceLeft;
  error2 = setpoint - distanceRight;
  double output1 = kp * error1 + ki * integral1 + kd * (error1 - lastError1);
  integral1 += error1;
  lastError1 = error1;
  double output2 = kp * error2 + ki * integral2 + kd * (error2 - lastError2);
  integral2 += error2;
  lastError2 = error2;
  int leftMotorSpeed = constrain(int(output1), -motorMapSpeed, motorMapSpeed);
  int rightMotorSpeed = constrain(int(output2), -motorMapSpeed, motorMapSpeed);
  runMotorByGivenSpeed(1, leftMotorSpeed);
  runMotorByGivenSpeed(2, rightMotorSpeed);

  delay(normalPIDDelayTime);
}


void testAllMotors(int delayTime) {
  runMotorByGivenSpeed(1, 200);
  runMotorByGivenSpeed(2, 0);
  delay(delayTime);


  runMotorByGivenSpeed(1, 0);
  runMotorByGivenSpeed(2, 200);
  delay(delayTime);

  runMotorByGivenSpeed(1, 200);
  runMotorByGivenSpeed(2, 200);
  delay(delayTime);

  runMotorByGivenSpeed(1, 0);
  runMotorByGivenSpeed(2, 0);
  delay(delayTime);


  runMotorByGivenSpeed(1, -200);
  runMotorByGivenSpeed(2, -200);
  delay(delayTime);

  runMotorByGivenSpeed(1, 0);
  runMotorByGivenSpeed(2, 0);
  delay(delayTime);

  runMotorByGivenSpeed(1, 200);
  runMotorByGivenSpeed(2, -200);
  delay(delayTime);

  runMotorByGivenSpeed(1, 0);
  runMotorByGivenSpeed(2, 0);
  delay(delayTime);

  runMotorByGivenSpeed(1, -200);
  runMotorByGivenSpeed(2, 200);
  delay(delayTime);

  runMotorByGivenSpeed(1, 0);
  runMotorByGivenSpeed(2, 0);
  delay(delayTime);
}


void stopAllMotors() {
  analogWrite(leftMotorA, LOW);
  analogWrite(leftMotorB, LOW);
  analogWrite(rightMotorA, LOW);
  analogWrite(rightMotorB, LOW);
}


void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


void getTOFValues(Adafruit_VL53L0X lox, int bus, int tofErr) {
  TCA9548A(bus);
  VL53L0X_RangingMeasurementData_t measure;

  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!

  if (bus == 1) {
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      readingF = measure.RangeMilliMeter - tofErr;
    }
  }

  if (bus == 2) {
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      readingR = measure.RangeMilliMeter - tofErr;
    }
  }

  if (bus == 7) {
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      readingL = measure.RangeMilliMeter - tofErr;
      // Serial.print("Distance (mm): "); Serial.println(readingL);
    } else {
      // Serial.println(" out of range ");
    }
  }
}


// TODO: Test
void moveForwardOneSquare() {
  stopAllMotors();
  runMotorByGivenSpeed(1, forwardSpeed);
  runMotorByGivenSpeed(2, forwardSpeed);
  delay(moveForwardTime);
  stopAllMotors();
}


// TODO: Test
void turnLeft90() {
  stopAllMotors();
  runMotorByGivenSpeed(1, -turnSpeedLeftMotor);
  runMotorByGivenSpeed(2, turnSpeedRightMotor);
  delay(turnLeftTime);
  stopAllMotors();
}


// TODO: Test
void turnRight90() {
  stopAllMotors();
  runMotorByGivenSpeed(1, turnSpeedLeftMotor);
  runMotorByGivenSpeed(2, -turnSpeedRightMotor);
  delay(turnRightTime);
  stopAllMotors();
}


void straightenRobot() {
  detectWallsAround();
  if (leftWallPresent && rightWallPresent && !(frontWallPresent)) {
    int errorToCorrectLeft = readingL - setpoint;
    int errorToCorrectRight = readingR - setpoint;
  }
}


void detectWallsAround() {
  getTOFValues(lox1, 1, errorF);
  getTOFValues(lox2, 2, errorR);
  getTOFValues(lox3, 7, errorL);
  if (readingL < leftWallDetectThreshold) {
    leftWallPresent = true;
    ledPatternLeftWallPresent();
  } else {
    leftWallPresent = false;
  }
  if (readingR < rightWallDetectThreshold) {
    rightWallPresent = true;
    ledPatternRightWallPresent();
  } else {
    rightWallPresent = false;
  }
  if (readingF < frontWallDetectThreshold) {
    frontWallPresent = true;
    ledPatternFrontWallPresent();
  } else {
    frontWallPresent = false;
  }
}


void ledPatternLeftWallPresent() {
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern * 2);
}


void ledPatternRightWallPresent() {
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern * 3);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern * 2);
}


void ledPatternFrontWallPresent() {
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern * 3);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, HIGH);
  delay(timeDelayLEDPattern);
  digitalWrite(inbuiltLED, LOW);
  delay(timeDelayLEDPattern);
}