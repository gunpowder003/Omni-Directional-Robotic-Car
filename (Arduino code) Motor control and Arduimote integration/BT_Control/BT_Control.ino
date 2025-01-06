//obtained from https://github.com/Haven-Lau/Arduimote-for-Arduino
#include <SoftEasyTransfer.h>
//#include <EasyTransfer.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 24);
SoftEasyTransfer ET; 
//EasyTransfer ET;
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
unsigned long x = 0;
unsigned long y = 0;
int leftPow = 0;
int rightPow = 0;

bool A;
bool C;
bool B;
bool D;
bool select;
bool f;
bool e;



void setup(){
    
    mySerial.begin(115200);
    //start the library, pass in the data details and the name of the serial port.
    ET.begin(details(mydata), &mySerial);
    Serial.begin(115200); 
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
  
         /*   //-**********************************************************
            if (x < 508 && y >= 515){ //upper left quadrant
              leftPow = (y - 515) - (508 - x);
              rightPow = (y - 515) + (508 - x);
            }else if (x <= 508 && y <= 515){ //lower left quadrant
              leftPow = -((515 - y) + (508 - x));
              rightPow = -((515 - y) - (508 - x));
            }else if (x >= 508 && y > 515){ //upper right quadrant
              leftPow = (y - 515) + (x - 508);
              rightPow = (y - 515) - (x - 508);
            }else if (x > 508 && y <= 515){ //lower right quadrant
              leftPow = (-(515 - y) + (x - 508));
              rightPow = (-(515 - y) - (x - 508));
            }*/
   }
     
    //delay(15);

}
