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
    // if (!(atWall))
    switch (direction) {
      case 1:

        if (!(goingToRefinery))
          if (safeZone)
            CheckIntersection();
          else {
            CheckWall();
            if (atWall)
              GrabBlock();
          }

        LineFollow();

        break;

      case 0:

        mdWheels.setSpeeds(100, -100);
        LineFollow();
        CheckIntersection();
    }
  }
}

void LineFollow(void) {

  double denom = 0;
  double d = 0;
  double error = 0;
  double num = 0;

  qtr.read(sensorValues);  // sensor values will be numbers between 0 and 2500

  // put your line localization code here
  for (uint8_t i = 0; i < 8; i++) {
    Sensor_value_unbiased[i] = sensorValues[i] - sensor_bias[i];
  }

  if (direction == 0)
    return;

  for (uint8_t i = 0; i < 8; i++) {
    num += Sensor_value_unbiased[i] * sensor_loc[i];
    denom += Sensor_value_unbiased[i];
  }

  d = num / denom;

  error = 2 - d;

  // put your line following code here
  m1c = base_speed + kpLineSensor * error;
  m2c = base_speed - kpLineSensor * error;

  mdWheels.setSpeeds(-m1c, m2c);  // send motor commands
}

void CheckIntersection(void) {

  double sum = 0;
  int numSensors;
  double avg = 0;

  if (direction == 1)
    numSensors = 5;
  else
    numSensors = 8;

  for (uint8_t i = 0; i < numSensors; i++)
    sum += Sensor_value_unbiased[i];

  avg = sum / numSensors;

  if (direction == 0) {
    if (avg > 1000) {
      Turn();
      mdWheels.setSpeeds(0, 0);
      delay(1000);
      mdWheels.setSpeeds(-75, 50);
      delay(250);
      mdWheels.setSpeeds(0, 0);
      direction = 1;
      goingToRefinery = true;
    }
  } else {
    if (avg > 1200) {
      if (currIntersection != activeIntersection) {
        if (onIntersection) {
          return;
        } else {
          onIntersection = true;
          return;
        }
      } else {
        Turn();
      }
    } else {
      if (onIntersection) {
        currIntersection += 1;
        onIntersection = false;
      }
    }
  }
}

void Turn(void) {

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

  t_old = micros() / 1000000.;
  t0 = micros() / 1000000.;

  // turn
  float angle;

  switch (activeIntersection) {
    case (1):
      angle = (M_PI / 6);
      break;
    case (2):
      angle = (M_PI / 10);
      break;
    case (3):
      angle = (M_PI / 12);
      break;
  };

  theta1_final = angle * (D / 2) / rw;
  theta2_final = -angle * (D / 2) / rw;

  // Set the desired velocity in radians/s
  omega1_des = 2;
  omega2_des = -omega1_des;

  myEnc1.write(0);
  myEnc2.write(0);

  while (true) {

    t = micros() / 1000000. - t0;  // Current time in seconds since start
    deltaT = t - t_old;            // Sample time

    // Read encoder counts and calculate wheel positions and velocities
    counts1 = myEnc1.read();
    counts2 = -myEnc2.read();

    theta1 = -counts1 * 360 / (countsPerRev * GearRatio) * (M_PI / 180);
    theta2 = -counts2 * 360 / (countsPerRev * GearRatio) * (M_PI / 180);

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

  safeZone = false;

  return;
}

void CheckWall(void) {

  if (analogRead(distSensor) > 170 && approach) {
    mdWheels.setSpeeds(0, 0);
    atWall = true;
    approach = false;

    GrabBlock();
  }

  else if (analogRead(distSensor) < 40) {
    approach = true;
    // CalcWormRate();
  }
}

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

void GrabBlock(void) {

  delay(1000);

  direction = 0;

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

int CheckBlock(void) {

  if (abs(analogRead(hallSensor)) > 100)
    return 1;

  // need to add check color
}

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

void CheckColor(void) {
}