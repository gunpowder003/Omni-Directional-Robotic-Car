/*
  Multple Serial test

 Receives from the main serial port, sends to the others.
 Receives from serial port 1, sends to the main serial (Serial 0).

 This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc

 The circuit:
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:

 created 30 Dec. 2008
 modified 20 May 2012
 by Tom Igoe & Jed Roach
 modified 27 Nov 2015
 by Arturo Guadalupi

 This example code is in the public domain.

 */


void setup() {
  // initialize both serial ports:
  Serial.begin(19200);
  Serial1.begin(19200);
}

void loop() {
  // read from port 1, send to port 0: (PHONE TO ARDUINO)
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 1: (ARDUINO TO PHONE)
  /*if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }*/
}
