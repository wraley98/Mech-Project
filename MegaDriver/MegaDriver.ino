// Load in libraries
#include "L298NMotorDriverMega.h"
#include "DualTB9051FTGMotorShieldUnoMega.h"
#include <PWMServo.h>
#include <QTRSensors.h>
#include <Encoder.h>

//// Pins ////

// Rack and Pinnion Motor
const int ENA = 33;
const int IN1 = 35;
const int IN2 = 37;
const int ENB = 42;
const int IN3 = 39;
const int IN4 = 41;

// Servo Motor Pin
const int servoMotor = 11;

// Distance Sensor assigment
const int distSensor = A1;

// Hall effect sensor
const int hallSensor = A2;

// Distance Sensor assigment
const int freqPin = A3;

// Switch pins
const int frontSwitchPin = 13;
const int backSwitchPin = 12;

// Color Sensor Pins
const int s0 = 28, s1 = 26, s2 = 30, s3 = 32;
int readPin = 24, LEDPin = 22;

//// Driver Set Up ////

// Rack and Pinnion motor shield
L298NMotorDriverMega mdRP(ENA, IN1, IN2, ENB, IN3, IN4);

// Wheel otors shield
DualTB9051FTGMotorShieldUnoMega mdWheels;

// Servo Motor shield
PWMServo armServo;

// Line Sensor driver
QTRSensors qtr;

// Encoder drivers
Encoder myEnc1(18, 19);
Encoder myEnc2(20, 21);

//// Sensor Setup ////

// Line Sensor Setup //
const uint8_t SensorCount = 8;       // # of sensors in reflectance array
uint16_t sensorValues[SensorCount];  //reflectance sensor readings
double kpLineSensor = 64;            //Proportional Gain for Line Following
double kdLineSensor = 2.0;           //Derivative Gain for line following
double prevLineError = 0;            //Previous error from line sensor
double tLineOld = 0;
int m1c = 0, m2c = 0;              //declare and initialize motor commands
int16_t Sensor_value_unbiased[8];  // unbiased sensor readings

/// !!!! Run TestLineSensor and adjust values on new surface!!!!
int16_t sensor_bias[8] = { 100, 152, 208, 100, 100, 152, 152, 152 };  // sensor biases
int16_t sensor_loc[8] = { 0, 0.8, 1.6, 2.4, 3.2, 4 };                 // distances between sensors

// Color Sensor

const int numSamples = 8;
float R[numSamples], G[numSamples], B[numSamples], C[numSamples];  // raw pulse time samples
float RF, GF, BF, CF;                                              // filtered data

// color ranges {R , G , B}
float blockRanges[3][3][2] = { { { 0.05, 0.25 }, { 0.15, 0.35 }, { 0.5, 0.7 } },
                               { { 0.55, 0.75 }, { 0.1, 0.4 }, { 0.15, 0.45 } },
                               { { 0.3, 0.69 }, { 0.41, 0.7 }, { 0.05, 0.41 } } };

String clrArr[3] = { "Blue", "Red", "Yellow" };

// Encoder Setup
double kpEncoder = 0.5;  //Proportional Gain for Trajectory Following
double GearRatio = 70;   // gear ratio
int countsPerRev = 64;   // encoder counts per Rev
double rw = 4.2;         // wheel radius in cm
double D = 26;           // distance between wheels in cm

// Distance Sensor
bool approach = false;
bool atWall = false;

//// Program Driven Variables ////

// determines whether the drive loop is active
bool active = false;
// determines whether the drive function is on its first loop
bool firstLoop = true;
// determines the direction the robot is driving ( 1 forward / 0 backward)
int direction = 1;
// determines if robot is in safe or danger zone
bool safeZone = true;
// Counts how many blocks have been picked up
int blockCount = 0;

// Worm Variables

// determines how long the robot has until the worm arrives
double wormTime = 0;
// maximum acceptable freq of the worm
int maxWormFreq = 340;
// determines if the worm rate has been checked
bool wormRateChecked = false;
// initial time that the worm time is based on
double initWormTime;

// Refinery Varibables

// determines if robot is at the refinery
bool atRefinery = false;
// determines if the robot is going to the refinery
bool goingToRefinery = false;

// Intersection variables

// Determines if intersection has been reached
bool intersetionReached = false;
// Defines what intersection is currently active
int activeIntersection = 1;
// Defines what intersection the robot is currently at
int currIntersection = 1;
// Checks whether line sensor is on intersection
bool onIntersection = false;

// Robot variables
// Nominal speed of robot
double base_speed = 75;
// RP motore speed
int rpMotorSpeed = 400;
// Tracks if robot has completed its turn
bool turned = false;

void setup() {
  // Computer Baud Rate
  Serial.begin(9600);
  // Uno Comm Baud Rate
  Serial2.begin(9600);

  // Initialize Drive Motors
  mdWheels.init();
  mdWheels.enableDrivers();

  // Initialize Rack and Pinion Motor
  mdRP.init();

  // Initialize Servo Motor
  armServo.attach(servoMotor);
  armServo.write(0);

  // Initializes Line Sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 53, 51, 47, 31, 29, 27, 25, 23 }, SensorCount);

  // switch pin setup
  pinMode(frontSwitchPin, INPUT);
  pinMode(backSwitchPin, INPUT);

  // Color sensor pin setup
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(readPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(s0, LOW);  // s1 and s0 choose frequency scaling
  digitalWrite(s1, HIGH);

  digitalWrite(LEDPin, HIGH);  //turn on LED
}

void loop() {

  if (Serial2.available() > 0) {
    String message = Serial2.readString();

    activeIntersection = message.toInt();

    active = true;
  }

  // If activation signal is sent from uno, startDriving
  if (active) {

    // direction (1 = forward // 0 = backward)
    switch (direction) {

        // if robot is going forward
      case 1:
        // if robot is not going to the refinery
        if (!(goingToRefinery)) {
          // if the robot is going to the safe zone
          if (safeZone) {
            // check for an intersection
            CheckIntersection();
          } else {
            // check for wall
            CheckWall();
            // grab the block if the robot is at the wall
            if (atWall) {
              // if (!(wormRateChecked)) {
              //   CalcWormRate();
              //   if (direction == 0)
              //     break;
              // } else if (millis() / 1000000. - initWormTime < wormTime) {
              //   direction = 0;
              //   base_speed = 200;
              //   break;
              // }
              GrabBlock();
            }
          }
        } else {
          // look for refinery wall
          CheckWall();
          // if the refinery has been reached
          if (atRefinery)
            // dispense the block
            DispenseBlock();
        }

        // follow the line
        LineFollow();

        break;

      // if the robot is moving backwards
      case 0:
        // move the robot backwards
        if (!(turned)) {
          mdWheels.setSpeeds(base_speed, -base_speed);
          delay(2000);
          Turn();
          base_speed = 75;
        } else
          mdWheels.setSpeeds(-100, 100);

        turned = true;
        // look for the line
        CheckIntersection();
        break;
    }
  }
}

// Drive Functions

/*
LineFollow

Retrieves line sensor values and updates the robots 
motors to allign the bot with the line. 
*/
void LineFollow(void) {

  double denom = 0;  // denominator for calculating distance from line
  double num = 0;    // numinator for calculating distance from line
  double d = 0;      // distance from center of line
  double error = 0;  // error between center of bot and center of line
  double derError = 0;
  unsigned long t = millis();
  double deltaT = t - tLineOld;

  // retrieve unbiased sensor values
  GetUnbiased();

  // if the robot is going backwards, return
  if (direction == 0)
    return;

  // add up the numerator and denomitor values
  for (uint8_t i = 0; i < 8; i++) {
    num += Sensor_value_unbiased[i] * sensor_loc[i];
    denom += Sensor_value_unbiased[i];
  }

  // location of the center line on the sensors
  d = num / denom;

  // error between the location of the center line and center robot
  if (goingToRefinery) {
    error = d - 2;
    // derError = (-prevLineError + error)/deltaT;
    // prevLineError = error;
  } else {
    error = 2 - d;
    // derError = (-prevLineError + error)/deltaT;
    // prevLineError = error;
  }
  // motor adjustments based on the error
  // m1c = base_speed + kpLineSensor * error + kdLineSensor * derError;
  // m2c = base_speed - kpLineSensor * error - kdLineSensor * derError;
  m1c = base_speed + kpLineSensor * error;
  m2c = base_speed - kpLineSensor * error;

  // send motor commands
  mdWheels.setSpeeds(-m1c, m2c);
  double tOld = t;
}
/*
CheckIntersection

Looks for intersections along the safe zone line 
or just the safe zone line.
*/
void CheckIntersection(void) {

  double sum = 0;  // initializes the sum of the sensor values
  int numSensors;  // initializes the number of sensors that will be used
  double avg = 0;  // initializes the average variable

  // retrieve the unbiased sensor values
  GetUnbiased();

  // if the robot is going forward, use the left half of the sensors
  if (direction == 1)
    numSensors = 5;
  // if the robot is going backwards, use all sensors
  else
    numSensors = 8;

  // add up all the unbiased sensors
  for (uint8_t i = 0; i < numSensors; i++)
    sum += Sensor_value_unbiased[i];

  // calculate the average of the sensor values
  avg = sum / numSensors;

  // if the robot is going backward
  if (direction == 0) {

    // if the sensor average is greater than 1000
    if (avg > 1000) {
      // reset speed to original
      base_speed = 75;
      // reset the current intersection
      currIntersection = 0;

      delay(250);
      // turn the robot
      Turn();
      // set the robot going forward
      direction = 1;
      // set the robot to going to the refinery
      goingToRefinery = true;
    }
  } else {
    // if the sensor average is greater than 1200
    if (avg > 1200) {
      // if the robot is not on the intersection with blocks
      if (currIntersection != activeIntersection) {
        // if the robot is currently on an intersection, return
        if (onIntersection) {
          return;
        }
        // set onIntersection to true and return
        else {
          onIntersection = true;
          return;
        }
      } else {
        // turn the robot
        Turn();
      }
    } else {
      // if the robot is on an intersection, increment the intersection
      // tracker and set the onIntersection to false
      if (onIntersection) {
        currIntersection += 1;
        onIntersection = false;
      }
    }
  }
}

/*
GetUnbiased

Retrieves the sensor values and calculates the 
the unbiased sensor values
*/
void GetUnbiased(void) {

  // retrieve sensor values
  qtr.read(sensorValues);

  // calculate the unbiased sensor values
  for (uint8_t i = 0; i < 8; i++) {
    Sensor_value_unbiased[i] = sensorValues[i] - sensor_bias[i];
  }
}
/*
Turn

Turns the robot based on what intersection the 
robot is currently on. 
*/
void Turn(void) {

  // stops bot
  mdWheels.setSpeeds(0, 0);

  // Time variables
  double t, t_old, deltaT, print_time, t0 = 0;

  // Control parameters
  double Kp = 0.6;           // Proportional Gain for Trajectory Following
  double Kp_straight = 0.3;  // Gain for straight-line correction

  // Encoder and motor variables
  long counts1 = 0, counts2 = 0;
  int m1c = 0, m2c = 0;  // Motor commands

  // Physical parameters
  double GearRatio = 70;
  int countsPerRev = 64;
  double rw = 5;    // Wheel radius in cm
  double D = 25.4;  // Distance between wheels in cm
  double xF = 50;   // Target distance in cm

  // Desired and actual state variables
  double theta1 = 0, theta1_old = 0, omega1 = 0;
  double theta2 = 0, theta2_old = 0, omega2 = 0;
  double theta1_des = 0, theta2_des = 0;
  double theta1_final, theta2_final;
  double omega1_des, omega2_des;

  // Motor voltage commands
  double V1m, V2m;

  // Set the time values
  t_old = micros() / 1000000.;
  t0 = micros() / 1000000.;

  float angle;  // initializes the angle that robot is turning to

  // determines the angle to turn by based
  // on the current intersection the bot is on
  switch (currIntersection) {
    // robot is leaving the wall
    case (-1):
      // resets the worm rate checked flag
      wormRateChecked = false;
      if (safeZone) {
        angle = 1.1 * M_PI;
      } else
        angle = M_PI;

      break;
    // robot is driving backwards from danger zone
    case (0):
      mdWheels.setSpeeds(-75, 75);
      // delay(500);
      mdWheels.setSpeeds(0, 0);
      angle = -6.9 * M_PI / 4.;
      break;
    // first intefrsection
    case (1):
      angle = M_PI / 10.;
      break;
    // second intersection
    case (2):
      angle = M_PI / 10;
      break;
    // third intersection
    case (3):
      angle = M_PI / 10.;
      break;
    // Fourth intersection
    case (4):
      angle = M_PI / 10.;
      break;
  };

  // initializes the final angle
  theta1_final = angle * (D / 2) / rw;
  theta2_final = -angle * (D / 2) / rw;

  // Set the desired velocity in radians/s
  omega1_des = 2;
  omega2_des = -omega1_des;

  // reset the encoders
  myEnc1.write(0);
  myEnc2.write(0);

  // infinite loop
  while (true) {

    t = micros() / 1000000. - t0;  // Current time in seconds since start
    deltaT = t - t_old;            // Sample time

    // Read encoder counts and calculate wheel positions and velocities
    counts1 = myEnc1.read();
    counts2 = -myEnc2.read();


    // update the angle of the wheels
    theta1 = -counts1 * 360 / (countsPerRev * GearRatio) * (M_PI / 180);
    theta2 = -counts2 * 360 / (countsPerRev * GearRatio) * (M_PI / 180);

    // set the angular speed
    omega1 = (theta1 - theta1_old) / deltaT;
    omega2 = (theta2 - theta2_old) / deltaT;

    // Update desired positions based on desired velocity and sample time
    theta1_des += omega1_des * deltaT;
    theta2_des += omega2_des * deltaT;

    // Compute the control signal (proportional control)
    V1m = Kp * (theta1_des - theta1);
    V2m = Kp * (theta2_des - theta2);

    // Add a correction term for straight-line motion based on wheel position error
    double correction = Kp_straight * (theta1 - theta2);
    V1m += correction;
    V2m -= correction;

    // if either angles are greater than the final angle, stop
    if (abs(theta1) >= abs(theta1_final) || abs(theta2) >= abs(theta2_final)) {
      break;
    }

    // Constrain motor commands within safe range
    V1m = constrain(V1m, -12, 12);
    V2m = constrain(V2m, -12, 12);
    m1c = 400 * V1m / 12;
    m2c = 400 * V2m / 12;

    // Set motor speeds
    mdWheels.setSpeeds(m1c, -m2c);

    // Update previous values
    t_old = t;
    theta1_old = theta1;
    theta2_old = theta2;
  }

  // if the robot is not driving backwards or leaving the refinery,
  // the bot is in the danger zone
  if (currIntersection != 0 || currIntersection != -1)
    safeZone = false;
  // else, the bot is in the safe zone
  else
    safeZone = true;

  return;
}

// Sensor Functions

/*
CheckWall

Determines if the robot is close to a wall.
*/
void CheckWall(void) {

  // if the distance sensor reading is too high, ignore the reading
  if (analogRead(distSensor) > 200)
    return;

  // if the reading is greater than 180, stop
  if (analogRead(distSensor) > 185 && approach) {

    // if the robot is going to the refinery,
    // set at reinery to true
    if (goingToRefinery) {
      mdWheels.setSpeeds(0, 0);
      atRefinery = true;
      return;
    }

    // stop the robot
    mdWheels.setSpeeds(0, 0);
    // set the robot at the wall is true
    atWall = true;
    approach = false;

    // grab the block
    // GrabBlock();
  } else if (analogRead(distSensor) < 85) {
    approach = true;
  }
}

/*
CalcWormRate

Calculates the current estimated time of arrival of the worm.
*/
void CalcWormRate(void) {

  double initWormTime = micros() / 1000000.;
  int rate1 = digitalRead(freqPin);

  if (rate1 > maxWormFreq) {
    base_speed = 200;
    direction = 0;
    return;
  }

  double finalWormTime = micros() / 1000000.;
  int rate2 = digitalRead(freqPin);

  double wormVelocity = (rate2 - rate1) / (finalWormTime - initWormTime);

  initWormTime = micros() / 1000000.;
  wormTime = (maxWormFreq - rate2) / wormVelocity;

  wormRateChecked = true;
}

// Arm Functions

void GrabBlock(void) {

  armServo.write(0);

  // send arm forward
  mdRP.setM1Speed(rpMotorSpeed);

  MoveArmForward();

  // stop the rp motor
  mdRP.setM1Brake(rpMotorSpeed);
  mdRP.setM1Speed(0);

  // drop arm
  armServo.write(175);

  delay(2000);

  CheckBlock();
}

void CheckBlock(void) {

  int result = CheckColor();

  if (abs(analogRead(hallSensor)) > 570 || abs(analogRead(hallSensor)) < 490 || result == 2) {
    PushBlock();
    return;
  } else if (result == -1) {
    activeIntersection += 1;
    direction = 0;
    currIntersection = -1;
    turned = false;
  } else
    PullBlock();
  if (blockCount == 2) {
    direction = 0;
    currIntersection = -1;
    turned = false;
  }
}

int CheckColor(void) {
  // standard deviation
  float rSTD = 0;
  float gSTD = 0;
  float bSTD = 0;

  GetVals();

  float rNorm[8];
  float gNorm[8];
  float bNorm[8];

  Normalize(R, rNorm);
  Normalize(G, gNorm);
  Normalize(B, bNorm);

  float rAvg = MovingAverage(rNorm);
  float gAvg = MovingAverage(gNorm);
  float bAvg = MovingAverage(bNorm);

  int clr = DetermineColor(rAvg, gAvg, bAvg);
}

float readPulse() {
  return 1 / ((pulseIn(readPin, LOW) + pulseIn(readPin, HIGH)) / 1000000.);
}

void GetVals(void) {

  // Take specified number of samples
  for (int i = 0; i < numSamples; i++) {
    // Select RED Filter
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    R[i] = readPulse();

    // Select BLUE Filter
    digitalWrite(s2, LOW);
    digitalWrite(s3, HIGH);
    B[i] = readPulse();

    // Select GREEN Filter
    digitalWrite(s2, HIGH);
    digitalWrite(s3, HIGH);
    G[i] = readPulse();

    // Select CLEAR Filter
    digitalWrite(s2, HIGH);
    digitalWrite(s3, LOW);
    C[i] = readPulse();
  }
}

void Normalize(float* arr, float* normArr) {

  for (int ii = 0; ii < numSamples; ii++)
    normArr[ii] = arr[ii] / C[ii];
}

float MovingAverage(float* arr) {
  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += arr[i] / numSamples;
  }
  return sum;
}

int DetermineColor(float r, float g, float b) {

  float block[3][2];

  // B == 0 / R == 1 / Y == 2
  for (int ii = 0; ii < 3; ii++) {

    float rLow = blockRanges[ii][0][0];
    float rHigh = blockRanges[ii][0][1];
    float gLow = blockRanges[ii][1][0];
    float gHigh = blockRanges[ii][1][1];
    float bLow = blockRanges[ii][2][0];
    float bHigh = blockRanges[ii][2][1];

    if (rLow < r && rHigh > r) {
      if (gLow < g && gHigh > g) {
        if (bLow < b && bHigh > b) {
          return ii;
        }
      }
    }
    return -1;
  }
}

void PushBlock(void) {

  // raise arm
  armServo.write(0);

  mdRP.setM1Speed(-rpMotorSpeed);

  delay(2500);

  // stop the rp motor
  mdRP.setM1Brake(rpMotorSpeed);
  mdRP.setM1Speed(0);

  // drop arm
  armServo.write(175);

  mdRP.setM1Speed(rpMotorSpeed);

  MoveArmForward();

  // stop the rp motor
  mdRP.setM1Brake(rpMotorSpeed);
  mdRP.setM1Speed(0);
}

void PullBlock(void) {

  mdRP.setM1Speed(-rpMotorSpeed);

  MoveArmBackward();

  // stop the rp motor
  mdRP.setM1Brake(rpMotorSpeed);
  mdRP.setM1Speed(0);

  // raise arm
  armServo.write(180);

  blockCount += 1;
}

void DispenseBlock(void) {

  mdWheels.setSpeeds(0, 0);

  armServo.write(0);

  mdRP.setM1Speed(rpMotorSpeed);

  MoveArmForward();

  mdRP.setM1Speed(-rpMotorSpeed);

  MoveArmBackward();

  armServo.write(160);

  mdRP.setM1Speed(rpMotorSpeed);

  delay(2000);

  // stop the rp motor
  mdRP.setM1Brake(rpMotorSpeed);
  mdRP.setM1Speed(0);

  mdRP.setM1Speed(-rpMotorSpeed);

  delay(1200);

  armServo.write(0);

  // stop the rp motor
  mdRP.setM1Brake(rpMotorSpeed);
  mdRP.setM1Speed(0);

  mdWheels.setSpeeds(base_speed, -base_speed);

  delay(4000);

  mdWheels.setSpeeds(0, 0);

  currIntersection = -1;

  Turn();

  mdWheels.setSpeeds(base_speed, -base_speed);

  delay(1000);

  mdWheels.setSpeeds(0, 0);

  Reset();
  // base_speed = 75;
  // safeZone = true;
  // currIntersection = 1;
  // // activeIntersection += 1;
  // goingToRefinery = false;
  // atRefinery = false;
  // direction = 1;
}

void MoveArmForward(void) {
  // initial read on pin
  int frontSwitchVal = digitalRead(frontSwitchPin);
  // read pin until pressed
  while (frontSwitchVal != 0) {
    frontSwitchVal = digitalRead(frontSwitchPin);
  }
}

void MoveArmBackward(void) {
  // initial read on pin
  int backSwitchVal = digitalRead(backSwitchPin);

  // read pin until pressed
  while (backSwitchVal != 0)
    backSwitchVal = digitalRead(backSwitchPin);
}

void Reset(void) {
  // Distance Sensor
  approach = false;
  atWall = false;

  //// Program Driven Variables ////

  // determines whether the drive loop is active
  active = "1";
  // determines whether the drive function is on its first loop
  firstLoop = true;
  // determines the direction the robot is driving ( 1 forward / 0 backward)
  direction = 1;
  // determines if robot is in safe or danger zone
  safeZone = true;
  // Counts how many blocks have been picked up
  blockCount = 0;

  // Worm Variables

  // determines how long the robot has until the worm arrives
  wormTime = 0;
  // maximum acceptable freq of the worm
  maxWormFreq = 340;
  // determines if the worm rate has been checked
  wormRateChecked = false;
  // initial time that the worm time is based on
  initWormTime;

  // Refinery Varibables

  // determines if robot is at the refinery
  atRefinery = false;
  // determines if the robot is going to the refinery
  goingToRefinery = false;

  // Intersection variables

  // Determines if intersection has been reached
  intersetionReached = false;
  // Defines what intersection the robot is currently at
  currIntersection = 1;
  // Checks whether line sensor is on intersection
  onIntersection = false;

  // Robot variables
  // Nominal speed of robot
  base_speed = 75;
  // RP motore speed
  rpMotorSpeed = 400;
  // Tracks if robot has completed its turn
  turned = false;
}
