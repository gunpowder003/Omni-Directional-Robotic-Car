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
int speed = 255; // how fast the motor is
int speed2  = 150;
int speed3 = 130;
int stop = 0;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorC, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(dirC, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  int inByte = 0;
  if (Serial1.available()) {
    inByte = Serial1.read();
    Serial.write(inByte);
  }

  if(inByte == '1') //front
  {
  digitalWrite(dirA, LOW);
  analogWrite(motorA, stop);
  digitalWrite(dirB, HIGH);
  analogWrite(motorB, speed);  //cw
  digitalWrite(dirC, LOW);
  analogWrite(motorC, speed); //ccw
  //delay(50);
  }

  if(inByte == '3')//backward
    {
  digitalWrite(dirA, LOW);
  analogWrite(motorA, stop);
  digitalWrite(dirB, LOW);
  analogWrite(motorB, speed);  //ccw
  digitalWrite(dirC, HIGH);
  analogWrite(motorC, speed); //cw
  //delay(50);
  }

if(inByte == '2')//right
    {
  digitalWrite(dirA, LOW);
  analogWrite(motorA, speed);
  digitalWrite(dirB, HIGH);
  analogWrite(motorB, speed2);  //cw
  digitalWrite(dirC, HIGH);
  analogWrite(motorC, speed3); //ccw
  //delay(50);
  }

  if(inByte == '4')//left
    {
  digitalWrite(dirA, HIGH);
  analogWrite(motorA, speed);
  digitalWrite(dirB, LOW);
  analogWrite(motorB, speed3);  
  digitalWrite(dirC, LOW);
  analogWrite(motorC, speed2);
  //delay(50);
  }
  
  if(inByte == '5') //rotate clockwise
    {
  digitalWrite(dirA, LOW);
  analogWrite(motorA, speed);
  digitalWrite(dirB, LOW);
  analogWrite(motorB, speed);  
  digitalWrite(dirC, LOW);
  analogWrite(motorC, speed); 
  //delay(50);
  }

  if(inByte == '6') //rotate counter-clockwise
    {
  digitalWrite(dirA, HIGH);
  analogWrite(motorA, speed);
  digitalWrite(dirB, HIGH);
  analogWrite(motorB, speed); 
  digitalWrite(dirC, HIGH);
  analogWrite(motorC, speed); 
  //delay(50);
  }

   if(inByte == '7') //stop
    {
  digitalWrite(dirA, LOW);
  analogWrite(motorA, stop);
  digitalWrite(dirB, LOW);
  analogWrite(motorB, stop); 
  digitalWrite(dirC, LOW);
  analogWrite(motorC, stop); 
  //delay(50);
  }
inByte = 0;
}
