/*
  State change detection (edge detection)

 Often, you don't need to know the state of a digital input all the time,
 but you just need to know when the input changes from one state to another.
 For example, you want to know when a button goes from OFF to ON.  This is called
 state change detection, or edge detection.

 This example shows how to detect when a button or button changes from off to on
 and on to off.

 The circuit:
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 * LED attached from pin 13 to ground (or use the built-in LED on
   most Arduino boards)

 created  27 Sep 2005
 modified 30 Aug 2011
 by Tom Igoe

This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/encAstateChange

 */

// this constant won't change:
const int  encA_A = 22;    // the pin that the pushbutton is attached to     // the pin that the LED is attached to

// Variables will change:
int encAcounter = 0;   // counter for the number of button presses
int encAstate = 0;         // current state of the button
int lastencAstate = 0;     // previous state of the button

void setup() {
  // initialize the button pin as a input:
  pinMode(encA_A, INPUT);
  // initialize serial communication:
  Serial.begin(9600);
}


void loop() {
  // read the pushbutton input pin:
  encAstate = digitalRead(encA_A);

  // compare the encAstate to its previous state
  if (encAstate != lastencAstate) {
    // if the state has changed, increment the counter
    if (encAstate == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      encAcounter++;
      Serial.print("number of button pushes:  ");
      Serial.println(encAcounter);
    } else {
      // if the current state is LOW then the button
      // wend from on to off:
      encAcounter++;
      Serial.print("number of button pushes:  ");
      Serial.println(encAcounter);
    }
    // Delay a little bit to avoid bouncing
    //delay(50);
  }
  // save the current state as the last state,
  //for next time through the loop
  lastencAstate = encAstate;

}









