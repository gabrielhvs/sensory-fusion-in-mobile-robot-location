// --- Pinout Encoders ---
#define PIN_ENCODER_R_CHANNEL_A 18
#define PIN_ENCODER_R_CHANNEL_B 19

#define PIN_ENCODER_L_CHANNEL_A 20
#define PIN_ENCODER_L_CHANNEL_B 21

// ==================================

// --- Pinout Ponte-H --- 
#define PIN_MOTOR_L1 22
#define PIN_MOTOR_L2 24
#define PIN_PWM_MOTOR_L 4

#define PIN_MOTOR_R1 26
#define PIN_MOTOR_R2 28
#define PIN_PWM_MOTOR_R 5

// ====================== 

// --- Variables ---
float rpm_left = 0;

long  time_left_channel_A = 0;
long time_left_channel_B = 0;
long  time_left = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(PIN_MOTOR_L1,OUTPUT);
  pinMode(PIN_MOTOR_L2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_L,OUTPUT);
  
  pinMode(PIN_MOTOR_R1,OUTPUT);
  pinMode(PIN_MOTOR_R2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_R,OUTPUT);

  
  pinMode(PIN_ENCODER_L_CHANNEL_A, INPUT);
  pinMode(PIN_ENCODER_L_CHANNEL_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANNEL_A), count_time_left_channel_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANNEL_B), count_time_left_channel_B, FALLING);
  
  digitalWrite(PIN_MOTOR_L1,1);
  digitalWrite(PIN_MOTOR_L2,0);
  analogWrite(PIN_PWM_MOTOR_L, 100);
}

void loop() {
 
    Serial.println(rpm_left);
}

void count_time_left_channel_A(){
  time_left_channel_A = micros();

  if(digitalRead(PIN_ENCODER_L_CHANNEL_B)){
    time_left = abs(time_left_channel_A - time_left_channel_B);
    rpm_left = ((1.0/(6.0*time_left)*1e6)*60.0)/75.0;
  }
}

void count_time_left_channel_B(){
  time_left_channel_B = micros();

  if(digitalRead(PIN_ENCODER_L_CHANNEL_A)){
    time_left = abs(time_left_channel_B - time_left_channel_A);
    rpm_left = ((1.0/(6.0*time_left)*1e6)*60.0)/75.0;
  }
}
