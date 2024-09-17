/*
RobotCode

Driver code for robot.

input:
  none

output:
  none

*/

// Libraries
#include "DualTB9051FTGMotorShield.h"

// Assign rx/tx pins
SoftwareSerial toMegaSerial(0, 1);

// Create motor shield
DualTB9051FTGMotorShield md;

void setup() {
  // Set up serial
  Serial.begin(9600);
  toMegaSerial.begin(9600);
}

void loop() {

  if (Serial.available() >= 2) {
    char input = Serial.read();

    if (input == "o" || input == "O")
      toMegaSerial.write(1);

    else if (input == "f" || input == "F")
      toMegaSerial.write(0);

    else
      Serial.write("Not a valid input");
  }
}
