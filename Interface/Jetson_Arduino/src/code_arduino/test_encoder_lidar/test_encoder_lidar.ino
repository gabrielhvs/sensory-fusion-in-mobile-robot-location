#define PIN_MOTOR_LIDAR 2
#define PIN_BOTAO 13
#define PIN_ENCODER_LIDAR 3

float pulses = 0;
float rpm = 0;
long long time_old = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_MOTOR_LIDAR, OUTPUT);
  pinMode(PIN_ENCODER_LIDAR,INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LIDAR), count_pulses, FALLING);
  

  analogWrite(PIN_MOTOR_LIDAR, 255);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - time_old >= 1000){
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LIDAR));

    rpm = ((pulses/ (millis()-time_old))*( 60.0 * 1e3));
    //Serial.print("dt: ");
    //Serial.print(long(millis()-time_old_left));
    Serial.print(" | RPM: ");
    Serial.println(rpm, DEC); 

    time_old = millis();
    pulses = 0;
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LIDAR), count_pulses, FALLING);
    //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B), count_pulses_left, FALLING);

  }
}


void count_pulses(){
  pulses++;
  //Serial.print(pulses);
}
