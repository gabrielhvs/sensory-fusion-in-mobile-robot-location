// --- Pinout Encoders ---
#define PIN_ENCODER_R_CHANEL_A 18
#define PIN_ENCODER_R_CHANEL_B 19

#define PIN_ENCODER_L_CHANEL_A 20
#define PIN_ENCODER_L_CHANEL_B 21


// ==================================

// --- Pinout Ponte-H --- 
#define PIN_MOTOR_L1 22
#define PIN_MOTOR_L2 24
#define PIN_PWM_MOTOR_L 4

#define PIN_MOTOR_R1 26
#define PIN_MOTOR_R2 28
#define PIN_PWM_MOTOR_R 5

// ====================== 

#define NUM_PULSES 300

// --- Variables ---
bool direction_motor_right;
bool direction_motor_left;

bool last_state_left_chA = false;
bool last_state_right_chA = false;

int pulses_left = 0;
int pulses_right = 0;

int rpm_right = 0;
int rpm_left = 0;
unsigned long long time_old_left = 0;
unsigned long long time_old_right = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  
  pinMode(PIN_MOTOR_L1,OUTPUT);
  pinMode(PIN_MOTOR_L2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_L,OUTPUT);
  
  pinMode(PIN_MOTOR_R1,OUTPUT);
  pinMode(PIN_MOTOR_R2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_R,OUTPUT);


  pinMode(PIN_ENCODER_R_CHANEL_A, INPUT);
  pinMode(PIN_ENCODER_R_CHANEL_B, INPUT);
  
  pinMode(PIN_ENCODER_L_CHANEL_A, INPUT);
  pinMode(PIN_ENCODER_L_CHANEL_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANEL_A), count_pulses_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANEL_B), count_pulses_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_A), count_pulses_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B), count_pulses_left, CHANGE);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  //Serial.print("Num. Pulses Right: ");
  //Serial.print(pulses_right);
  //Serial.print(" | Num. Pulses Left: ");
  //Serial.print(pulses_left);
  //Serial.print(" | RPM Right: ");
  //Serial.println(rpm_right);  
  //Serial.print(" | RPM Left: ");
  //Serial.println(rpm_left);  
  //rpm_right = ((1.0/NUM_PULSES)*60*1000)/(millis()-time_old_right);
  //rpm_left = ((1.0/NUM_PULSES)*60*1000)/(millis()-time_old_left);

  if(millis() - time_old_left >= 100){
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANEL_A));
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANEL_B));
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_A));
    detachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B));

    rpm_left =  60 *( pulses_left / NUM_PULSES ) * (1000 / millis()-time_old_left);
    time_old_left = millis();
    pulses_left = 0;

    Serial.print(" | RPM Left: ");
    Serial.println(rpm_left, DEC); 
    
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANEL_A), count_pulses_right, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANEL_B), count_pulses_right, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_A), count_pulses_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANEL_B), count_pulses_left, CHANGE);

  }
  
}

void count_pulses_left(){
  
  int state_left_chA = digitalRead(PIN_ENCODER_L_CHANEL_A);

  if(!last_state_left_chA && state_left_chA){
    int state_left_chB = digitalRead(PIN_ENCODER_L_CHANEL_B);
    if(!state_left_chB && direction_motor_left)
      direction_motor_left = false;
    else if(state_left_chB && !direction_motor_left)
      direction_motor_left = true;  
    pulses_left++;
  }

  last_state_left_chA = state_left_chA;

  //pulses_left = (!direction_motor_left)? pulses_left+1 : pulses_left-1;

}

void count_pulses_right(){
  
  int state_right_chA = digitalRead(PIN_ENCODER_R_CHANEL_A);

  if(!last_state_right_chA && state_right_chA){
    int state_right_chB = digitalRead(PIN_ENCODER_R_CHANEL_B);
    if(!state_right_chB && direction_motor_right)
      direction_motor_right = false;
    else if(state_right_chB && !direction_motor_right)
      direction_motor_right = true; 
    pulses_right++;
  }

  last_state_right_chA = state_right_chA;

  //pulses_right = (!direction_motor_right)? pulses_right+1 : pulses_right-1;
  
}
