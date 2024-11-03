#include <SoftwareSerial.h>

// Assign rx/tx pins
SoftwareSerial megaSerial(2, 3);

void setup() {

  Serial.begin(9600);
  megaSerial.begin(9600);
}

void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if (input.equals("1")) {
      megaSerial.write("1");
      Serial.println("Drive started");
    } else if (input.equals("0")) {
      megaSerial.write("0");
      Serial.println("Drive ended");
    } else {
      Serial.println("Not a valid input, please type:\n1 to drive or 0 to stop\n");
    }
  }

}
