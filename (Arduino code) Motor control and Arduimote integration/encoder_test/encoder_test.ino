/*
 * Encoder example sketch
 * by Andrew Kramer
 * 1/1/2016
 *
 * Records encoder ticks for each wheel
 * and prints the number of ticks for
 * each encoder every 500ms
 *
 */

// pins for the encoder inputs
#define A_ENCODER_A 2 
#define A_ENCODER_B 4
#define B_ENCODER_A 3 
#define B_ENCODER_B 5
#define C_ENCODER_A 20 
#define C_ENCODER_B 21

// variables to store the number of encoder pulses
// for each motor
volatile signed long ACount = 0;
volatile signed long BCount = 0;
volatile signed long CCount = 0;


void setup() {
  pinMode(A_ENCODER_A, INPUT);
  pinMode(A_ENCODER_B, INPUT);
  pinMode(B_ENCODER_A, INPUT);
  pinMode(B_ENCODER_B, INPUT);
  pinMode(C_ENCODER_A, INPUT);
  pinMode(C_ENCODER_B, INPUT);

  
  // initialize hardware interrupts
  attachInterrupt(0, AEncoderEvent, CHANGE);
  attachInterrupt(1, BEncoderEvent, CHANGE);
  attachInterrupt(3, CEncoderEvent, CHANGE);
  
  Serial.begin(115200);
}

void loop() {
  Serial.print("A Count: ");
  Serial.println(ACount);
  Serial.print("B Count: ");
  Serial.println(BCount);  
  Serial.print("C Count: ");
  Serial.println(CCount);
  Serial.print("\n");
  

  delay(500);
}

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



