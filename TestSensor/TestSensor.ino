#include <QTRSensors.h>
#include "DualTB9051FTGMotorShieldUnoMega.h"
#include <SoftwareSerial.h>

// create motor objects
QTRSensors qtr;
DualTB9051FTGMotorShieldUnoMega md;

const uint8_t sensorCount = 8;
uint16_t sensorValues[sensorCount];
int16_t Sensor_value_unbiased[8];
const int16_t sensor_bias[8] = { 156, 156, 104, 156, 156, 208, 104, 156 };  // !!!! Must updated sesnor bias !!!!!!
const int16_t sensor_loc[8] = { 0, 0.8, 1.6, 2.4, 3.2, 4 };
double num = 0;
double denom = 0;
double d;
String direction = "forward";
int speed = 100;

// initializes error
double error;
void setup() {
  Serial.begin(9600);
  // Serial1.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 53, 51, 47, 31, 29, 27, 25, 23 }, sensorCount);  ///23,25,27,29,31,53,49,47} ,sensorCount); //52 , 50 , 48 , 46

  md.init();
  md.enableDrivers();
}

void loop() {
  WallSensor();
  // LineSensor();
}

void WallSensor(void){

  Serial.println(analogRead(A1));

}

void LineSensor(void) {
  qtr.read(sensorValues);  // sensor values will be numbers between 0 and 2500

  for (uint8_t i = 0; i < 8; i++)
    Sensor_value_unbiased[i] = sensorValues[i] - sensor_bias[i];

  // collect numerator and denominator for localization code
  for (uint8_t i = 0; i < 8; i++) {
    num += Sensor_value_unbiased[i] * sensor_loc[i];
    denom += Sensor_value_unbiased[i];
  }

  d = num / denom;  // line localization

  error = 2 - d;  // calculates the error

  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(Sensor_value_unbiased[i]);
    Serial.print(" ");
  }

  Serial.print(" d:");
  Serial.print(d);
  Serial.print(" Error:");
  Serial.println(error);

  delay(3000);
}
