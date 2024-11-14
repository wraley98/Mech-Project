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

// Distance Sensor assigment
const int freqPin = 7;

// hall effect sensor
const int hallSensor = A2;

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
double kpLineSensor = 60;            //Proportional Gain for Line Following
double base_speed = 75;              //Nominal speed of robot
int m1c = 0, m2c = 0;                //declare and initialize motor commands
int16_t Sensor_value_unbiased[8];    // unbiased sensor readings

/// !!!! Run TestLineSensor and adjust values on new surface!!!!
int16_t sensor_bias[8] = { 428, 480, 816, 428, 480, 480, 592, 592 };  // sensor biases
int16_t sensor_loc[8] = { 0, 0.8, 1.6, 2.4, 3.2, 4 };                 // distances between sensors

// storage vals for Line Sensor


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
String active = "1";
// determines whether the drive function is on its first loop
bool firstLoop = true;
// determines the direction the robot is driving ( 1 forward / 0 backward)
int direction = 1;
// determines if robot is in safe or danger zone
bool safeZone = true;

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

  // Initializes Line Sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 53, 51, 47, 31, 29, 27, 25, 23 }, SensorCount);

  // frequency pin
  pinMode(freqPin, INPUT_PULLUP);
}

void loop() {

  // If activation signal is sent from uno, startDriving
  if (active.equals("1")) {

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
            if (atWall)
              GrabBlock();
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
        mdWheels.setSpeeds(100, -100);
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
  error = 2 - d;

  // motor adjustments based on the error
  m1c = base_speed + kpLineSensor * error;
  m2c = base_speed - kpLineSensor * error;

  // send motor commands
  mdWheels.setSpeeds(-m1c, m2c);
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
      // stop the robot
      mdWheels.setSpeeds(0, 0);
      // reset the current intersection
      currIntersection = 0;
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

  // let the robot drive for .25 seconds
  delay(250);

  // stops bot
  mdWheels.setSpeeds(0, 0);

  // Time variables
  double t, t_old, deltaT, print_time, t0 = 0;

  // Control parameters
  double Kp = 0.6;           // Proportional Gain for Trajectory Following
  double Kp_straight = 0.3;  // Gain for straight-line correction

  // Encoder and motor variables
  long counts1, counts2;
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
    // robot is leaving the refinery
    case (-1):
      angle = M_PI / 1.2;
      break;
    // robot is driving backwards from danger zone
    case (0):
      // continue driving backwards for 0.3 seconds
      mdWheels.setSpeeds(-75, 75);
      delay(300);
      mdWheels.setSpeeds(0, 0);
      angle = M_PI / 4.;
      break;
    // first intersection
    case (1):
      angle = M_PI / 5.3;
      break;
    // second intersection
    case (2):
      angle = M_PI / 10.;
      break;
    // third intersection
    case (3):
      angle = M_PI / 12.;
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
  if (analogRead(distSensor) > 180) {
   
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

    // grab the block
    GrabBlock();
  }
}

/// !!!! Need to Implement!!!!
void CalcWormRate(void) {


  /// NEED TO ADD EVAC TIME

  long pulseTimeLow = 0;
  long pulseTimeHigh = 0;
  long pulseSum = 0;
  long timeOutDuration = 20000;
  float freq[2];

  double t0 = millis();
  double t;

  for (int ii = 0; ii < 2; ii++) {
    pulseTimeLow = pulseIn(freqPin, LOW, timeOutDuration);
    pulseTimeHigh = pulseIn(freqPin, HIGH, timeOutDuration);
    pulseSum = pulseTimeLow + pulseTimeHigh;

    freq[ii] = 1 / (float(pulseSum)) * 1000000.0;

    if (ii == 0)
      delay(500);
    else
      t = millis();
  }
}

// Sand dune Functions

/// !!!! Need to Implement !!!!
void GrabBlock(void) {

  delay(1000);

  direction = 0;
  atWall = false;

  // rotate arm up
  // armServo.write(90);

  // move arm forward
  // mdRP.setM1Speed(400);

  // stop moving
  // mdRP.setM1Brake(400);
  // mdRP.setM1Speed(0);

  // rotate arm down
  // armServo.write(0);
}

/// !!!! Need to Implement !!!!
int CheckBlock(void) {

  if (abs(analogRead(hallSensor)) > 100)
    return 1;

  // need to add check color
}

/// !!!! Need to Implement !!!!
void PushBlock(void) {
  // rotate arm up
  // armServo.write(90);

  // move arm backward
  // mdRP.setM1Speed(-400);

  // stop moving
  // mdRP.setM1Brake(400);
  // mdRP.setM1Speed(0);

  // rotate arm down
  // armServo.write(0);

  // move arm forward
  // mdRP.setM1Speed(400);
}

/// !!!! Need to Implement !!!!
void DispenseBlock(void) {

  mdWheels.setSpeeds(0, 0);
  delay(1000);

  mdWheels.setSpeeds(75, -75);
  delay(1500);

  mdWheels.setSpeeds(0, 0);

  currIntersection = -1;

  Turn();
  safeZone = true;
  currIntersection = 1;
  activeIntersection += 1;
  goingToRefinery = false;
  atRefinery = false;
}

/// !!!! Need to Implement !!!!
void CheckColor(void) {
}