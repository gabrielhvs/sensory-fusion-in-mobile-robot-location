
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // set the data rate for the SoftwareSerial port
  Serial3.begin(115200);
}

void loop() {
  
}

void serialEvent3(){
    
    Serial.write(Serial3.read());
}
