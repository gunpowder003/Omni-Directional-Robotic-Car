

void setup(){
Serial.begin(115200);
}

void loop(){
  char inByte;
if (Serial.available()) {
  inByte=Serial.read();
Serial.print(inByte);
}
//delay(500);
}


