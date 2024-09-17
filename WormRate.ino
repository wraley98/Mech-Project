/*
WormRate

Class to calculate worm rate and store previous rate. 

input:
  ledPin ~ analog pin for the led sensor

output:
  none 

*/
// Load libraries
#include <List.hpp>

// Create variables
int wormEta;         // time until worm will arive to dune location
float initWormTime;  // initial time worm rate was taken


class WormRate() {

  void init(int ledPin) {
    wormEta(ledPin);
  }

  /*
  Determines the worm rate by taking samples of the LED pin.

  input:
  ledPin ~ pin that led is currently using

  output:
  none
  */
  void wormRateEta(int ledPin) {

    // determine frequency of worm led
    double firstRead = analogRead(ledPin);
    initWormTime = millis();

    delay(500);

    double finalRead = analogRead(ledPin);
    float timeFinal = millis();

    float rate = (finalRead - firstRead) / (timeFinal - initWormTime);

    // calculate time to worm arrives

    // !!! need current lane !!!
  }

  //  !!! needs header !!!
  bool CheckWorm() {
    float timeElapsed = millis() - initWormTime;

    // check if worm rate is too high or low

    return 0
  }

  float GetWormTime(); // !!! need to complete !!!
}
}
