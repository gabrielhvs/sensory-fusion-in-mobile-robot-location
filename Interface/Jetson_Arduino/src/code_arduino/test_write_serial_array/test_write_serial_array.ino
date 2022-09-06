byte msg_serial[5] = {253};
byte rpm_left_roda = 145;
byte rpm_right_roda = 177;
byte clock_left = 1;
byte clock_right = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  msg_serial[1] = rpm_left_roda;
  msg_serial[2] = rpm_right_roda;
  msg_serial[3] = clock_left;
  msg_serial[4] = clock_right;
  
  write_msg_serial();
}

void write_msg_serial(){
    for(int i=0; i<5;i++)
      Serial.write(msg_serial[i]);  
}
