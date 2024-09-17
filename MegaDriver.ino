/*
MegaDriver

Driver code for mega board. Waits for input from uno:

1 ~ start run
2 ~ end run

input:
  none

output:
  none

*/

// Load libraries
#include "DualTB9051FTGMotorShield.h"
#include <Hashtable.h>
#include <Arduino.h>

// Assign rx/tx pins
SoftwareSerial toUnoSerial(0, 1);

// Create motor shield
DualTB9051FTGMotorShield md;

// Create Pins

// Analog Pins
const int magSensor = A0;
const int frontLineSensor = A1;
const int backLineSensor = A2;
const int frontDistanceSensor = A3;
const int backDistanceSensor = A4;
const int ledPin = A5;
const int colorSensor = A6;
// const int motorSensor; need to add

// Digital Pins

// Create Variables
// dynamic variables
bool active = 0;
bool direction = 0;
int lineSensor;
int onCourse;
bool inDangerZone = 0;
bool startUp = 1;
bool driving = 0;
int numBlocks = 0;
int activeLane = 1;
int currLane = 0;
WormRate wormRateObject;
int distanceSensor;
// steady variables
const int maxWormRate;
Hashtable<String, List<int>> motorPinArr;

void setup() {
  // Set up serial
  Serial.begin(9600);
  toUnoSerial.begin(9600);

  // create servo pin

  // Save motorPins to motorPinArray

  // set digitial pin modes

  // Create worm rate object
}

void loop() {

  if (Serial.available() >= 2) {

    int input = Serial.read();

    if (input)
      active = 1;

    else 
      active = 0;
  }

  if(active){

    if(startUp){
      Drive(direction , motorPinArr);
      startUp = 0;
    }



  }
}
