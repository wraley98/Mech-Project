// Load in libraries
#include "L298NMotorDriverMega.h"
#include "DualTB9051FTGMotorShieldUnoMega.h"
#include <PWMServo.h>
#include <QTRSensors.h>
#include <Encoder.h>

//// Pins ////

// Rack and Pinnion Motor
const int ENA = 24;
const int IN1 = 26;
const int IN2 = 28;
const int ENB = 22;
const int IN3 = 30;
const int IN4 = 36;

// Servo Motor Pin
const int servoMotor = 11;


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
Encoder myEnc1(7, 9);
Encoder myEnc2(8, 10);

//// Sensor Setup ////

// Line Sensor Setup //
const uint8_t SensorCount = 8;                                     // # of sensors in reflectance array
uint16_t sensorValues[SensorCount];                                //reflectance sensor readings
double t, t0, print_time = 0;                                      // declare some time variables
double kpLineSensor = 60;                                          //Proportional Gain for Line Following
double base_speed = 75;                                            //Nominal speed of robot
int m1c = 0, m2c = 0;                                              //declare and initialize motor commands
int16_t Sensor_value_unbiased[8];                                  // unbiased sensor readings
int16_t sensor_bias[8] = { 92, 152, 432, 92, 92, 100, 260, 152 };  // sensor biases
int16_t sensor_loc[8] = { 0, 0.8, 1.6, 2.4, 3.2, 4 };              // distances between sensors
// stroage vals for Line Sensor
double num;
double denom;
double d;
double error;

// Encoder Setup
double kpEncoder = 0.5;  //Proportional Gain for Trajectory Following
double GearRatio = 70;   // gear ratio
int countsPerRev = 64;   // encoder counts per Rev
double rw = 4.2;         // wheel radius in cm
double D = 26;           // distance between wheels in cm


// Distance Sensor assigment
const int distSensor = A1;


bool approach = false;

//// Program Driven Variables ////

// determines whether the drive loop is active
String active = "1";
// determines whether the drive function is on its first loop
bool firstLoop = true;
// determines the direction the robot is driving
char direction = 'forward';
// Determines if intersection has been reached
bool intersetionReached = false;


void setup() {
  // Computer Baud Rate
  Serial.begin(9600);
  // Uno Comm Baud Rate
  Serial1.begin(9600);

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
}

void loop() {

  // If activation signal is sent from uno, startDriving
  if (active.equals("1")) {

    switch (direction) {
      case 'forward':

        CheckIntersection();

        LineFollow();

        break;
    }
  }
}

void LineFollow(void) {

  t = micros() / 1000000. - t0;

  qtr.read(sensorValues);  // sensor values will be numbers between 0 and 2500
  num = 0;
  denom = 0;
  // put your line localization code here
  for (uint8_t i = 0; i < 8; i++) {
    Sensor_value_unbiased[i] = sensorValues[i] - sensor_bias[i];
  }

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

  for (uint8_t i = 0; i < 4; i++)
    num += Sensor_value_unbiased[i];

  int numSensors;

  if (direction == 'forward')
    numSensors = 4;

  double avg = num / numSensors;

  if (avg > 1700)
    Turn();
}

void Turn(void) {

  // stops bot
  mdWheels.setSpeeds(0, 0);

  // calculates final rotation distance
  double theta1_final = ((M_PI / 2) * D / 2) / rw;
  double theta2_final = -((M_PI / 2) * D / 2) / rw;

  double omega1_des = 4 / rw, omega2_des = 4 / rw;

  double theta1_des = 0, theta2_des = 0;
  double theta1_old = 0, theta2_old = 0;

  double theta1, theta2;
  double omega1, omega2;

  double V1m, V2m;

  double t_old = micros() / 1000000.;
  double t0 = micros() / 1000000.;

  while (theta1 < theta1_final) {
    double t = micros() / 1000000. - t0;
    double deltaT = t - t_old;  // sample time
    t_old = t;

    // read counts
    long counts1 = myEnc1.read();
    long counts2 = myEnc2.read();

    // calculate your position and velocity here
    theta1 = -counts1 * 360 / (countsPerRev * GearRatio) * (M_PI / 180);
    theta2 = -counts2 * 360 / (countsPerRev * GearRatio) * (M_PI / 180);

    omega1 = (theta1 - theta1_old) / deltaT;
    omega2 = (theta2 - theta2_old) / deltaT;

    theta1_des = theta1_des + omega1_des * deltaT;
    theta2_des = theta2_des + omega2_des * deltaT;

    V1m = kpEncoder * (theta1_des - theta1);
    V2m = kpEncoder * (theta2_des - theta2);

    V1m = constrain(V1m, -10, 10);
    V2m = constrain(V2m, -10, 10);

    m1c = 400 * V1m / 10;
    m2c = 400 * V2m / 10;

    mdWheels.setSpeeds(m1c, m2c);

    theta1_old = theta1;
    theta2_old = theta2;
  }
}
