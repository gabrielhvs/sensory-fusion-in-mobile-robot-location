// --- Pinout Encoders ---
#define PIN_ENCODER_R_CHANNEL_A 20
#define PIN_ENCODER_R_CHANNEL_B 21

#define PIN_ENCODER_L_CHANNEL_A 18
#define PIN_ENCODER_L_CHANNEL_B 19


// ==================================

// --- Pinout Ponte-H --- 
#define PIN_MOTOR_L1 22
#define PIN_MOTOR_L2 24
#define PIN_PWM_MOTOR_L 4

#define PIN_MOTOR_R1 26
#define PIN_MOTOR_R2 28
#define PIN_PWM_MOTOR_R 5

// ====================== 

#define TIMEOUT 1e9
#define ARCO 0.011 // 11mm
// --- Variables ---
float rpm_left = 0;

long  time_left_channel_A = 0;
long time_left_channel_B = 0;
long  time_left = 0;

bool channelA = 0;
bool channelB = 0;

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
 
  Serial.print(channelA);
  Serial.print(',');
  Serial.println(channelB);
  
   
}

void count_time_left_channel_A(){
  //time_left_channel_A = micros();
  channelA = !channelA;
}

void count_time_left_channel_B(){
  //time_left_channel_B = micros();
  channelB = !channelB;
}
