//Bluetooth control obtained from https://github.com/Haven-Lau/Arduimote-for-Arduino
#include <SoftEasyTransfer.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(50, 24);
SoftEasyTransfer ET; 

//encoder pin set
#define A_ENCODER_A 2 
#define A_ENCODER_B 4
#define B_ENCODER_A 3 
#define B_ENCODER_B 5
#define C_ENCODER_A 20 
#define C_ENCODER_B 21

// Easy transfer struct
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int fi;
  int se;
  int th;
  int fo;
};

RECEIVE_DATA_STRUCTURE mydata;

int first = 0, second = 0, third = 0, forth = 0;
int i = 0;
int x = 0;
int y = 0;
int w = 0;
double leftPow = 0;
double rightPow = 0;

bool A;
bool C;
bool B;
bool D;
bool select;
bool f;
bool e;

//encoder declaration
volatile signed long ACount = 0;
volatile signed long BCount = 0;
volatile signed long CCount = 0;

//Ultrasonic sensor pin
const int pinsensA = 45;
const int pinsensB = 41;
const int pinsensC = 49;

// the PWM pin the motor is attached to and the initial value
int motorA = 9;
int motorB = 10;
int motorC = 11;
double mA = 0, mB = 0, mC = 0;

//direction pin for motor
int dirA = 8;
int dirB = 13;
int dirC = 12;

//ultrasonic sensor initial value
int sensorA = 0,
    sensorB = 0, 
    sensorC = 0;
    
//charcter from Pi
char inByte, oldByte;

//mode selector switch pin for the robot
const int modePin = 30;
const int modeLedPin = 52;

//desired x, y and angle velocity
double Vx, Vy, Vw;
//max speed parameter in 8 bit (255)
int maxspeed = 255;
int maxspeedrotation= 150;
// mode choice variable for BT and camera control; 0 for bluetaooth, 1 for camera
int mode = 0;
//camera tracking variable
double cx,cy,cw;
// the setup routine runs once when you press reset:
void setup() {
  mySerial.begin(115200);
  //start the library, pass in the data details and the name of the serial port.
  ET.begin(details(mydata), &mySerial);
  //serial setup
  //if run on windows choose 115200
  Serial.begin(115200);
  //mode switch initialization
  pinMode(modePin, INPUT_PULLUP);
  pinMode(modeLedPin, OUTPUT);
  //encoder initialization
  pinMode(A_ENCODER_A, INPUT);
  pinMode(A_ENCODER_B, INPUT);
  pinMode(B_ENCODER_A, INPUT);
  pinMode(B_ENCODER_B, INPUT);
  pinMode(C_ENCODER_A, INPUT);
  pinMode(C_ENCODER_B, INPUT);
  //attach interrupt for encoder
  //attachInterrupt(0, AEncoderEvent, CHANGE);
  //attachInterrupt(1, BEncoderEvent, CHANGE);
  //attachInterrupt(3, CEncoderEvent, CHANGE);

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
  //sensor reading
  sensorA = SensorReading(pinsensA);
  sensorB = SensorReading(pinsensB);
  sensorC = SensorReading(pinsensC);
  //Serial.println();
  //encoder print
  //EncoderPrint(pinsensA, ACount);
  //EncoderPrint(pinsensB, BCount);
  //EncoderPrint(pinsensC, CCount);
  //Serial.println();

  ReadMode();
  if(mode == 0)
  BluetoothMotorControl();
  
  else if(mode == 1)
  CameraTrackingMotorControl();
}
//end of main loop



//User defined function for Ultrasonic sensor
//part of ultrasonic sensor reading fucntion
/*long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}*/
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
  //inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  return cm;
/*
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
  */
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
int ReadMode(void)
{
  int modestate = 0;
  modestate = digitalRead(modePin);
  if (modestate == HIGH) {
    // turn LED on, set mode:
    digitalWrite(modeLedPin, HIGH);
    mode = 0;
  } else {
    // turn LED off:
    digitalWrite(modeLedPin, LOW);
    mode =1;
  }
}


//bluetooth control user defined function
void   BluetoothMotorControl(void)
{
   if( ET.receiveData()){
      
          // assign software serial struct objects to variables
          // message is 4 bits long
          // protocol = first[XXXX XXXX] second[XXYY YYYY] third[YYYY RLUD] forth[SFE0 0000]
          
          first = mydata.fi;
          second = mydata.se;
          third = mydata.th;
          forth = mydata.fo;
     
          //decode analog x, y and button values 
          x = first * 4 + second / 64;
          y = (second % 64) * 16 + (third / 16);
          A = !bitRead(third,3);
          C = !bitRead(third,2);
          B = !bitRead(third,1);
          D = !bitRead(third,0);
          select = !bitRead(forth,7);
          f = !bitRead(forth, 6);
          e = !bitRead(forth,5);
    
      #if 0 // change 0 to 1 for serial debugging
      
          Serial.print(" x: ");
          Serial.print(x);
          Serial.print(" y: ");
          Serial.print(y);
          Serial.print(" A: ");
          Serial.print(A);
          Serial.print(" B: ");
          Serial.print(B);
          Serial.print(" C: ");
          Serial.print(C);
          Serial.print(" D: ");
          Serial.print(D);
          Serial.print(" select: ");
          Serial.print(select);
          Serial.print(" f: ");
          Serial.print(f);
          Serial.print(" e: ");
          Serial.println(e);
          
      #endif
  sensorA = SensorReading(pinsensA);
  sensorB = SensorReading(pinsensB);
  sensorC = SensorReading(pinsensC);
  SensorCompare(sensorC, sensorA, sensorB);
          
   MotorControl(x, y, w);  
   }
}
//end of bluetooth control

//camera tracking function
void CameraTrackingMotorControl(void)
{ 

  if (Serial.available()>0)
  {
    inByte = Serial.read();
   //Serial.print(inByte);
    //Serial.println();
    if(inByte == 's') //stop
      {cy=0;cx=0;cw=0;} 
    else if(inByte == 'a') //slow forward
      {cx=0;cy=0.3;cw=0;}
    else if(inByte == 'b')//fast forward
      {cx=0;cy=0.6;cw=0;}
    else if(inByte == 'e')//slow reverse
      {cx=0;cy=-0.3;cw=0;}
    else if(inByte == 'f')//fast reverse
      {cx=0;cy=-0.6;cw=0;}
    else if(inByte == 'k')//slow clock wise
      {cy=0;cx=0;cw=0.15;}
    else if(inByte == 'l')//fast cclockwise
      {cy=0;cx=0;cw=0.15;}
    else if(inByte == 'i')//slow ccw
      {cy=0;cx=0;cw=-0.15;}  
    else if(inByte == 'j')//fast ccw
      {cy=0;cx=0;cw=-0.15;}
    else if(inByte == 'c') //slow left
      {cy=0;cx=0.1;cw=0;}  
    else if(inByte == 'd')//fast left
      {cy=0;cx=0.2;cw=0;}
    else if(inByte == 'g')//slow right
      {cy=0;cx=-0.1;cw=0;}
    else if(inByte == 'h')//fast right
      {cy=0;cx=-0.2;cw=0;}
    // if(inByte == 'm')//rotate to find human
    //  {cy=0;cx=0;cw=0.2;}
    oldByte = inByte;
  }
  else if (Serial.available()<=0)
  {//Serial.print(oldByte);
    //Serial.println();
    if(oldByte == 's') //stop
      {cy=0;cx=0;cw=0;} 
    else if(oldByte == 'a') //slow forward
      {cx=0;cy=0.3;cw=0;}
    else if(oldByte == 'b')//fast forward
      {cx=0;cy=0.6;cw=0;}
    else if(oldByte == 'e')//slow reverse
      {cx=0;cy=-0.3;cw=0;}
    else if(oldByte == 'f')//fast reverse
      {cx=0;cy=-0.6;cw=0;}
    else if(oldByte == 'k')//slow clock wise
      {cy=0;cx=0;cw=0.15;}
    else if(oldByte == 'l')//fast cclockwise
      {cy=0;cx=0;cw=0.15;}
    else if(oldByte == 'i')//slow ccw
      {cy=0;cx=0;cw=-0.15;}  
    else if(oldByte == 'j')//fast ccw
      {cy=0;cx=0;cw=-0.15;}
    else if(oldByte == 'c') //slow left
      {cy=0;cx=0.2;cw=0;}  
    else if(oldByte == 'd')//fast left
      {cy=0;cx=0.4;cw=0;}
    else if(oldByte == 'g')//slow right
      {cy=0;cx=-0.2;cw=0;}
    else if(oldByte == 'h')//fast right
      {cy=0;cx=-0.4;cw=0;}
   // else if(oldByte == 'm')//rotate to find human
   //   {cy=0;cx=0;cw=0.9;} 
  }
  sensorA = SensorReading(pinsensA);
  sensorB = SensorReading(pinsensB);
  sensorC = SensorReading(pinsensC);
  SensorCompare(sensorB, sensorA, sensorC);
  /*Serial.print(sensorC);
  Serial.print(" = C    ");
  Serial.print(sensorA);
  Serial.print(" = A    ");
  
  Serial.print(sensorB);
  Serial.println(" = B");*/
  
  MotorControl(-cy, cx, cw); //invert x and y to get the same as bluetooth controller

  
}
//end of camera tracking function

//sensor compare function
void SensorCompare(int leftsensor, int centresensor, int rightsensor)
{
  if (centresensor<=15)
  {if (leftsensor<rightsensor)//strafe right 
    { x=509;y=250;w=0;
      cx=0.4;cy=0;cw=0;
     }
    else if(rightsensor<leftsensor)//strafe left
    { x=509;y=250;w=0;
      cx=-0.4;cy=0;cw=0;
     }
  }
  else if (centresensor>=15 && rightsensor<30)
  { x=750;y=509;w=0;
    cx=-0.4;cy=0;cw=0;
  }
  else if (centresensor>=15 && leftsensor<30)
  { x=250;y=509;w=0;
    cx=0.4;cy=0;cw=0;
  }
     
}
//end of sensor compare function

//Motor control function
void MotorControl(double x, double y, double w)
{   double mA, mB, mC; 
  /*Serial.print("  x:");
  Serial.print(x);
  Serial.println();
    Serial.print("  y:");
  Serial.print(y);
  Serial.println();
     Serial.print("  w:");
  Serial.print(w);
  Serial.println();*/
  if (mode == 0)//for bluetooth conversion
  {
    Vy = (double(x) - 512)/512;
    Vx = (double(y) - 512)/512;
    Vx = -Vx;

       if (A == 1)
          maxspeedrotation = maxspeedrotation +3;
       if (maxspeedrotation>255)
          maxspeedrotation = 255;
        if (C == 1)
        maxspeedrotation = maxspeedrotation -3;
        if (maxspeedrotation<0)
          maxspeedrotation = 0;
       
      if(D ==1)
      Vw = (-1*(double(maxspeedrotation))/double(maxspeed));
      else if(B ==1)
      Vw = (+1*(double(maxspeedrotation))/double(maxspeed));
      else
      Vw = w;
  }
  else if (mode == 1)//for camera parameter conversion
  {
    Vx = double(x);
    Vy = double(y);
    Vw = double(w);
  }
    /*  Serial.print("  Vx:");
  Serial.print(Vx);
  Serial.println();
     Serial.print("  Vy:");
  Serial.print(Vy);
  Serial.println();*/


  
  
  mA = 0.58*Vx - 0.33*Vy + 0.33*Vw;
  mB = -0.58*Vx - 0.33*Vy + 0.33*Vw;
  mC = 0.67*Vy + 0.33*Vw;

  if(mA >= 0)
    digitalWrite(dirA, LOW);
  else if (mA < 0)
    {
    mA = -mA;
    digitalWrite(dirA, HIGH);
    }
  mA *= maxspeed;
  analogWrite(motorA, mA);
  //Serial.print("motor A:");
  //Serial.print(mA);
            
  if(mB >= 0)
    digitalWrite(dirB, LOW);
  else if (mB < 0)
  {
    mB = -mB;
    digitalWrite(dirB, HIGH);
  }
  mB *= maxspeed;
  analogWrite(motorB, mB);
  //Serial.print("  motor B:");
  //Serial.print(mB);
           
  if(mC >= 0)
    digitalWrite(dirC, LOW);
  else if (mC < 0)
  {
    mC = -mC;
    digitalWrite(dirC, HIGH);
   }
   
  mC *= maxspeed;
  analogWrite(motorC, mC);
  
  /*Serial.print("  motor C:");
  Serial.print(mC);
  Serial.println();
  Serial.println();
  */
  //delay(100);
  

}
//end of motor control function



