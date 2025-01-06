//obtained from https://github.com/Haven-Lau/Arduimote-for-Arduino
#include <SoftEasyTransfer.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(50, 24);
SoftEasyTransfer ET; 

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
double leftPow = 0;
double rightPow = 0;

bool A;
bool C;
bool B;
bool D;
bool select;
bool f;
bool e;

//desired x, y and angle velocity
double Vx, Vy, Vw;
//max speed parameter in 8 bit (255
int maxspeed = 255;

// the PWM pin the motor is attached to
int motorA = 9;
int motorB = 10;
int motorC = 11;
double mA = 0, mB = 0, mC = 0;
//direction pin for motor
int dirA = 8;
int dirB = 13;
int dirC = 12;


void setup(){
    mySerial.begin(115200);
    //start the library, pass in the data details and the name of the serial port.
    ET.begin(details(mydata), &mySerial);
    Serial.begin(115200);
  //motor initialization
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  pinMode(motorC, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(dirC, OUTPUT);
}

void loop(){

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
    
      #if 1 // change 0 to 1 for serial debugging
      
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
  
           Vx = (double(x) - 512)/512;
            Vy = (double(y) - 512)/512;
            Serial.print(Vx);
            Serial.print("  ");
            Serial.print(Vy);
            Serial.println();
              double mA, mB, mC;
              Vw=0;
            if(D ==1)
            Vw = 1;
            if(B ==1)
            Vw = -1;
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
            /*error = ACount - (-BCount);
            
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
            }*/
            Serial.print("  motor B:");
            Serial.print(mB);
            
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
   
     
    //delay(15);

}
