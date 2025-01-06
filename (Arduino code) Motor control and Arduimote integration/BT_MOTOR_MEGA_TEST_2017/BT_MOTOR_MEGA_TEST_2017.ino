#define A_ENCODER_A 2 
#define A_ENCODER_B 4
#define B_ENCODER_A 3 
#define B_ENCODER_B 5
#define C_ENCODER_A 20 
#define C_ENCODER_B 21

//encoder declaration
volatile signed long ACount = 0;
volatile signed long BCount = 0;
volatile signed long CCount = 0;

//Ultrasonic sensor pin
const int pinsensA = 45;
const int pinsensB = 41;
const int pinsensC = 49;

// the PWM pin the motor is attached to
int motorA = 9;
int motorB = 10;
int motorC = 11;

//direction pin for motor
int dirA = 8;
int dirB = 13;
int dirC = 12;

//variable initalization
int sensorA = 0,
    sensorB = 0, //ultrasonic sensor
    sensorC = 0;

//mode selector switch for the robot
int modePin = 40;


double x=0, y=0, w=0; //desired x, y and angle velocity
int  maxspeed = 100, stop = 0; //max speed parameter in 8 bit (255)
int mode = 0; // mode choice for BT and camera control; 1 for bluetooth, 0 for camera

// the setup routine runs once when you press reset:
void setup() {
  //serial setup
  Serial.begin(115200);
  //mode switch initialization
  pinMode(modePin, INPUT);
  //encoder initialization
  pinMode(A_ENCODER_A, INPUT);
  pinMode(A_ENCODER_B, INPUT);
  pinMode(B_ENCODER_A, INPUT);
  pinMode(B_ENCODER_B, INPUT);
  pinMode(C_ENCODER_A, INPUT);
  pinMode(C_ENCODER_B, INPUT);
  //attach interrupt for encoder
  attachInterrupt(0, AEncoderEvent, CHANGE);
  attachInterrupt(1, BEncoderEvent, CHANGE);
  attachInterrupt(3, CEncoderEvent, CHANGE);

  //motor initialization
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorC, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(dirC, OUTPUT);

}

// the loop routine runs over and over again forever:
void loop() 
{
  //digitalWrite(dirB, HIGH);
  //analogWrite(motorB, 80);
  //sensor reading
  sensorA = SensorReading(pinsensA);
  sensorB = SensorReading(pinsensB);
  sensorC = SensorReading(pinsensC);
  Serial.println();
  //encoder print
  EncoderPrint(pinsensA, ACount);
  EncoderPrint(pinsensB, BCount);
  EncoderPrint(pinsensC, CCount);
  Serial.println();

  //mode = ReadModePin(modePin);
  //if(mode == 1)
  BluetoothControl(sensorA, sensorB, sensorC);
  //else if(mode == 0)
  //CameraTracking(sensorA, sensorB, sensorC);
}
//end of main loop



//User defined function for Ultrasonic sensor
//part of ultrasonic sensor reading fucntion
long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}
//part of ultrasonic sensor reading fucntion
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

//user defined function for ultrasonic sensor reading
long SensorReading(int sensor)
{
  long duration, inches, cm;
  pinMode(sensor, OUTPUT);
  digitalWrite(sensor, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor, HIGH);
  delayMicroseconds(5);
  digitalWrite(sensor, LOW);
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(sensor, INPUT);
  duration = pulseIn(sensor, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

  if(sensor == 45)
  Serial.print("sensor A: ");
  else if(sensor == 41)
  Serial.print("sensor B: ");
  else if(sensor == 49)
  Serial.print("sensor C: ");
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm ");
}
//end of user defined functions for ultrasonic sensor


//Intterrupt event for encoder reading
// encoder event for the interrupt call
void AEncoderEvent() {
  if (digitalRead(A_ENCODER_A) == HIGH) {
    if (digitalRead(A_ENCODER_B) == LOW) {
      ACount++;
    } else {
      ACount--;
    }
  } else {
    if (digitalRead(A_ENCODER_B) == LOW) {
      ACount--;
    } else {
      ACount++;
    }
  }
}

  
 void BEncoderEvent() {
  if (digitalRead(B_ENCODER_A) == HIGH) {
    if (digitalRead(B_ENCODER_B) == LOW) {
      BCount++;
    } else {
      BCount--;
    }
  } else {
    if (digitalRead(B_ENCODER_B) == LOW) {
      BCount--;
    } else {
      BCount++;
    }
  }
 }
  
 
 void CEncoderEvent() {
  if (digitalRead(C_ENCODER_A) == HIGH) {
    if (digitalRead(C_ENCODER_B) == LOW) {
      CCount++;
    } else {
      CCount--;
    }
  } else {
    if (digitalRead(C_ENCODER_B) == LOW) {
      CCount--;
    } else {
      CCount++;
    }
  }
}
//print the encoder value to serial
void EncoderPrint(int pin, int Count)
{
  if(pin == pinsensA)
  Serial.print("A Count: ");
  else if(pin == pinsensB)
  Serial.print("  B Count: ");
  else if(pin == pinsensC)
  Serial.print("  C Count: ");  
  Serial.print(Count);
}
//end of encoder intterrupt events


//mode decision function
int ReadModePin(int m)
{
  int outcome;
  if(digitalRead(m) == HIGH)
  outcome = 1;
  else if(digitalRead(m) == LOW)
  outcome = 0;
  return outcome;
}


//bluetooth control user defined function
void BluetoothControl(int sA, int sB, int sC)
{
  int inByte = 0;
  if (Serial1.available()) {
    inByte = Serial1.read();
    Serial.write(inByte);
  }
  if(inByte == '1')
  {y=1;x=0;w=0;}
  else if(inByte == '3')
  {y=-1;x=0;w=0;}
  else if(inByte == '4')
  {y=0;x=-1;w=0;}
  else if(inByte == '2')
  {y=0;x=1;w=0;}
  else if(inByte == '5')
  {y=0;x=0;w=1;}
  else if(inByte == '6')
  {y=0;x=0;w=-1;}
  else if(inByte == '7')
  {y=0;x=0;w=0;}
  else if(inByte == '8')
  {y=1;x=1;w=0;}  if(inByte == '1')
  {y=1;x=0;w=0;}
  else if(inByte == '3')
  {y=-1;x=0;w=0;}
  else if(inByte == '4')
  {y=0;x=-1;w=0;}
  else if(inByte == '2')
  {y=0;x=1;w=0;}
  else if(inByte == '5')
  {y=0;x=0;w=1;}
  else if(inByte == '6')
  {y=0;x=0;w=-1;}
  else if(inByte == '7')
  {y=0;x=0;w=0;
  ACount = 0; BCount = 0; CCount=0;
  }
  else if(inByte == '8')
  {y=1;x=1;w=0;}
  
  MotorControl(x, y, w);
  

}
//end of bluetooth control

//camera tracking function
void CameraTracking(int sA, int sB, int sC)
{
  SetMotorSpeed();
  MotorControl(x, y, w);

}
//end of camera tracking function


//Motor control and speed selsction function
//motor speed function
int SetMotorSpeed(void)
{

}

//Motor control function
void MotorControl(double x, double y, double w)
{
  double error, error1, error2;
  double mA, mB, mC;
  
  mA = 0.58*x - 0.33*y + 0.33*w;
  mB = -0.58*x - 0.33*y + 0.33*w;
  mC = 0.67*y + 0.33*w;
  
if(mA >= 0)
  digitalWrite(dirA, LOW);
else if (mA < 0)
{
  mA = -mA;
  digitalWrite(dirA, HIGH);
}
mA *= maxspeed;
analogWrite(motorA, mA);
Serial.print("motor A:");
Serial.print(mA);


if(mB >= 0)
  digitalWrite(dirB, LOW);
else if (mB < 0)
{
  mB = -mB;
  digitalWrite(dirB, HIGH);
}
mB *= maxspeed;
analogWrite(motorB, mB);
error = ACount - (-BCount);

if(error<0)
{
  error = -error;
  mB += (error/0.015)/(255);
  digitalWrite(dirB, HIGH);
  analogWrite(motorB, mB);
}
if(error>=0)
{
  mB += (error/0.015)/(255);
  digitalWrite(dirB, LOW);
  analogWrite(motorB, mB);
}
Serial.print("  motor B:");
Serial.print(mB);
if(ACount > (-BCount))
{
  mB += ((ACount -(-BCount))/(2.55));
  digitalWrite(dirB, LOW);
  analogWrite(motorB, mB);}
else if(ACount < (-BCount))
{
  mB += ((ACount -(-BCount))/(2.55));
  digitalWrite(dirB, HIGH);
  analogWrite(motorB, mB);}
else if(ACount == (-BCount))
;
if(mC >= 0)
  digitalWrite(dirC, LOW);
else if (mC < 0)
{
  mC = -mC;
  digitalWrite(dirC, HIGH);
}
mC *= maxspeed;
analogWrite(motorC, mC);
Serial.print("  motor C:");
Serial.print(mC);
Serial.println();
Serial.println();

}



//end of motor control function



