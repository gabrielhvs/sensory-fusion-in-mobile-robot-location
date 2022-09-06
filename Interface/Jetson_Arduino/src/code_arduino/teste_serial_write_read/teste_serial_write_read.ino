void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  
}

void serialEvent3(){
  Serial.write(Serial3.read());
}
