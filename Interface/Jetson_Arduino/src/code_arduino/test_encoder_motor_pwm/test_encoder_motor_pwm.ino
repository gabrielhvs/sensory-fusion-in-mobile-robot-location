// --- Pinout Encoders ---
#define PIN_ENCODER_L_CHANNEL_A 18
#define PIN_ENCODER_L_CHANNEL_B 19

#define PIN_ENCODER_R_CHANNEL_A 20
#define PIN_ENCODER_R_CHANNEL_B 21


// ==================================

// --- Pinout Ponte-H --- 
#define PIN_MOTOR_L1 22
#define PIN_MOTOR_L2 24
#define PIN_PWM_MOTOR_L 4

#define PIN_MOTOR_R1 26
#define PIN_MOTOR_R2 28
#define PIN_PWM_MOTOR_R 5

// ====================== 

// --- Serial --- 

#define SIZE_BUFFER 9
#define BEGIN_MSG_01 0
#define BEGIN_MSG_02 5
#define SIZE_MSG_01 4
#define SIZE_MSG_01 4

// =====================


#define TIMEOUT 1e9
#define ARCO 0.011 // 11mm

// --- Variables ---

float rpm_left = 0;
int motor_pwm_left = 0;
 
float rpm_right = 0;
int motor_pwm_right = 0;

long long time_left_channel_A = 0;
long long time_left_channel_B = 0;

long long time_right_channel_A = 0;
long long time_right_channel_B = 0;

size_t bytes_received;
char msg_buffer[SIZE_BUFFER+1];
char msg_01[5];
char msg_02[5];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(PIN_MOTOR_L1,OUTPUT);
  pinMode(PIN_MOTOR_L2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_L,OUTPUT);
  
  pinMode(PIN_MOTOR_R1,OUTPUT);
  pinMode(PIN_MOTOR_R2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_R,OUTPUT);
  set_speed_motor(0,0);
  
  pinMode(PIN_ENCODER_L_CHANNEL_A, INPUT);
  pinMode(PIN_ENCODER_L_CHANNEL_B, INPUT);

  pinMode(PIN_ENCODER_R_CHANNEL_A, INPUT);
  pinMode(PIN_ENCODER_R_CHANNEL_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANNEL_A), count_time_left_channel_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANNEL_B), count_time_left_channel_B, FALLING);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANNEL_A), count_time_right_channel_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANNEL_B), count_time_right_channel_B, FALLING);
}

void loop() {
  Serial.print("RPM Right: ");
  Serial.print(rpm_right);
  Serial.print(" | RPM Left: ");
  Serial.println(rpm_left);

  bytes_received = Serial.readBytesUntil('\n',msg_buffer,SIZE_BUFFER);
  if(bytes_received > 0){
    //Serial.println(msg_buffer);
    decode_msg_csv(msg_buffer);
  }

  set_speed_motor(motor_pwm_left, motor_pwm_right);
  
  if(abs(time_left_channel_A - time_left_channel_B) >= TIMEOUT)
    rpm_left = 0;
  else
    rpm_left = (((ARCO/abs(time_left_channel_A - time_left_channel_B))*1e6)*60/(0.07*3.14*2))/(2840/180);
  
  if(abs(time_right_channel_A - time_right_channel_B) >= TIMEOUT)
    rpm_right = 0;
  else
    rpm_right = (((ARCO/abs(time_right_channel_A - time_right_channel_B))*1e6)*60/(0.07*3.14*2))/(2840/180);
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
  Serial.print(motor_speed_R);
  Serial.print(" | ");
  Serial.println(motor_speed_L);
  analogWrite(PIN_PWM_MOTOR_R,abs(motor_speed_R));
  analogWrite(PIN_PWM_MOTOR_L,abs(motor_speed_L));
}

void write_motor(int pin_motor_A, int pin_motor_B){
  digitalWrite(pin_motor_A, HIGH);
  digitalWrite(pin_motor_B, LOW);
}

void decode_msg_csv(char *msg_buffer_csv){
  for(int i = 0; i < SIZE_MSG_01;i++){
    msg_01[i] = msg_buffer[BEGIN_MSG_01+i];
    msg_02[i] = msg_buffer[BEGIN_MSG_02+i];
  }
  
  motor_pwm_right = atoi(msg_01);
  motor_pwm_left = atoi(msg_02);
}

void count_time_left_channel_A(){
  time_left_channel_A = micros();
}

void count_time_left_channel_B(){
  time_left_channel_B = micros();
}

void count_time_right_channel_A(){
  time_right_channel_A = micros();
}

void count_time_right_channel_B(){
  time_right_channel_B = micros();
}
