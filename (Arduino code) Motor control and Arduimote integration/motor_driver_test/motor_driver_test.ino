/*
 Fade

 This example shows how to fade an motor on pin 9
 using the analogWrite() function.

 The analogWrite() function uses PWM, so if
 you want to change the pin you're using, be
 sure to use another PWM capable pin. On most
 Arduino, the PWM pins are identified with 
 a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

 This example code is in the public domain.
 */

int motorA = 9;
int motorB = 10;
int motorC = 11;
int dirA = 8;// the PWM pin the motor is attached to
int dirB = 13;
int dirC = 12;
int i=0; 
int speed = 150;    // how fast the motor is
int fadeAmount = 2;    // how many points to fade the motor by

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorC, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(dirC, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  speed = 150;
  for(i=0;i<75;i++)
  {
  digitalWrite(dirA, LOW);
  analogWrite(motorA, speed);
  digitalWrite(dirB, LOW);
  analogWrite(motorB, speed);
  digitalWrite(dirC, LOW);
  analogWrite(motorC, speed);
  speed=speed - fadeAmount;
  delay(50);
  }
  speed = 150;
   for(i=0;i<75;i++)
  {
  digitalWrite(dirA, HIGH);
  analogWrite(motorA, speed);
  digitalWrite(dirB, HIGH);
  analogWrite(motorB, speed);
  digitalWrite(dirC, HIGH);
  analogWrite(motorC, speed);
  speed=speed - fadeAmount;
  delay(50);
  }
 delay(4000);
}
