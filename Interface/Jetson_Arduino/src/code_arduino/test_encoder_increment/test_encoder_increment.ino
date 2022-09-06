// --- Pinout Encoders ---
#define PIN_ENCODER_R_CHANEL_A 20
#define PIN_ENCODER_R_CHANEL_B 21

#define PIN_ENCODER_L_CHANEL_A 18 //
#define PIN_ENCODER_L_CHANEL_B 19 //


// ==================================

// --- Pinout Ponte-H --- 
#define PIN_MOTOR_L1 22
#define PIN_MOTOR_L2 24
#define PIN_PWM_MOTOR_L 4

#define PIN_MOTOR_R1 26
#define PIN_MOTOR_R2 28
#define PIN_PWM_MOTOR_R 5

// ====================== 

#define NUM_PULSES 75.0

// --- Variables ---
bool direction_motor_right;
bool direction_motor_left;

bool last_state_left_chA = false;
bool last_state_right_chA = false;

float pulses_left = 0;
float pulses_right = 0;

float rpm_right = 0;
float rpm_left = 0;
unsigned long long time_old_left = 0;
unsigned long long time_old_right = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(PIN_MOTOR_L1,OUTPUT);
  pinMode(PIN_MOTOR_L2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_L,OUTPUT);
  
  pinMode(PIN_MOTOR_R1,OUTPUT);
  pinMode(PIN_MOTOR_R2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_R,OUTPUT);
  
  pinMode(PIN_ENCODER_L_CHANEL_A, INPUT);
  //pinMode(PIN_ENCODER_L_CHANEL_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_A), count_pulses_left, FALLING);
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B), count_pulses_left, FALLING);
}

void loop() {
  

  if(millis() - time_old_left >= 1000){
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_A));
    //detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B));

    rpm_left = ((pulses_left)*( 60.0 * 1000.0))/ (millis()-time_old_left);
    //Serial.print("dt: ");
    //Serial.print(long(millis()-time_old_left));
    Serial.print(" | RPM Left: ");
    Serial.println(pulses_left, DEC); 

    time_old_left = millis();
    //pulses_left = 0;
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_A), count_pulses_left, FALLING);
    //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B), count_pulses_left, FALLING);

  }

  set_speed_motor(0,0);
  
}

void count_pulses_left(){
  
  pulses_left = pulses_left+1 ;

}

void set_speed_motor(int motor_speed_L, int motor_speed_R){

  if(motor_speed_R >= 0)
    write_motor(PIN_MOTOR_R1,PIN_MOTOR_R2);
  else
    write_motor(PIN_MOTOR_R2,PIN_MOTOR_R1);

  if(motor_speed_L >= 0)
    write_motor(PIN_MOTOR_L1,PIN_MOTOR_L2);
  else
    write_motor(PIN_MOTOR_L2,PIN_MOTOR_L1);
  
  if(motor_speed_R > 255 || motor_speed_R < -255)
    motor_speed_R = 255;
  if(motor_speed_L > 255 || motor_speed_L < -255)
    motor_speed_L = 255;
  //Serial.print(motor_speed_R);
  //Serial.print(" | ");
  //Serial.println(motor_speed_L);
  analogWrite(PIN_PWM_MOTOR_R,abs(motor_speed_R));
  analogWrite(PIN_PWM_MOTOR_L,abs(motor_speed_L));
}

void write_motor(int pin_motor_A, int pin_motor_B){
  digitalWrite(pin_motor_A, HIGH);
  digitalWrite(pin_motor_B, LOW);
}
