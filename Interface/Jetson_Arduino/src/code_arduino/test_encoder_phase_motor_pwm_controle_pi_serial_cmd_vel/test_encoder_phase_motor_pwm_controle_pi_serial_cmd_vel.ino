// --- Pinout Encoders ---
#define PIN_ENCODER_L_CHANNEL_A 18
#define PIN_ENCODER_L_CHANNEL_B 19

#define PIN_ENCODER_R_CHANNEL_A 21
#define PIN_ENCODER_R_CHANNEL_B 20


// ==================================

// --- Pinout Ponte-H ---
#define PIN_MOTOR_L1 22
#define PIN_MOTOR_L2 24
#define PIN_PWM_MOTOR_L 4

#define PIN_MOTOR_R1 26
#define PIN_MOTOR_R2 28
#define PIN_PWM_MOTOR_R 5

// ======================

// ------------- LiDAR -------------

#define PIN_PWM_MOTOR_LIDAR 3

#define TIMEOUT_ENCODERS 5000 // Mínima velocidade +-30 rpm
// --- Serial ---


#define SIZE_BUFFER 14
#define SIZE_BUFFER_RPM 9 //14
#define BEGIN_MSG_01 0
#define BEGIN_MSG_02 5
#define BEGIN_MSG_03 10
#define SIZE_MSG_01 4
#define SIZE_MSG_01 4

// =====================


// --- Variables ---


int rpm_lidar = 0;
double Kp_lidar = 0.5;
double error_lidar = 0;
int out_lidar = 0;

float rpm_left = 0;
float last_rpm_left = 0;
int rpm_left_roda = 0;
int clock_wize_left = 1;
int motor_pwm_left = 0;

float rpm_right = 0;
float last_rpm_right = 0;
int rpm_right_roda = 0;
int clock_wize_right = 1;
int motor_pwm_right = 0;

long  time_left_channel_A = 0;
long  time_left_channel_B = 0;
long  dt_left = 0;
long  dt_left_last = 0;

long  time_right_channel_A = 0;
long  time_right_channel_B = 0;
long  dt_right = 0;
long  dt_right_last = 0;


size_t bytes_received;
char msg_buffer_rpm[SIZE_BUFFER_RPM + 1];
char msg_01[5];
char msg_02[5];
char msg_03[5];

double kp_left = 3.740;
double ki_left = 0.0227;
double kd_left = 0;

unsigned long currentTime_left, previousTime_left;
double elapsedTime_left;
double error_left;
double lastError_left;
int set_rpm_left = 0;
double input_left, output_left;
double cumError_left, rateError_left;

double kp_right = 4.87;
double ki_right = 0.0336;
double kd_right = 0;

unsigned long currentTime_right, previousTime_right;
double elapsedTime_right;
double error_right;
double lastError_right;
int set_rpm_right = 0;
double input_right, output_right;
double cumError_right, rateError_right;
//float Ki = 10.5562;
//float Kd = 0.00046;

long ant_T = millis();

byte msg_serial_motor_rpms[5] = {253};

void setup() {
  Serial.begin(115200);  // USB serial
  Serial3.begin(115200);  // XV-11 LDS data

  pinMode(PIN_PWM_MOTOR_LIDAR, OUTPUT);

  pinMode(PIN_MOTOR_L1, OUTPUT);
  pinMode(PIN_MOTOR_L2, OUTPUT);
  pinMode(PIN_PWM_MOTOR_L, OUTPUT);

  pinMode(PIN_MOTOR_R1, OUTPUT);
  pinMode(PIN_MOTOR_R2, OUTPUT);
  pinMode(PIN_PWM_MOTOR_R, OUTPUT);
  set_speed_motor(0, 0);

  pinMode(PIN_ENCODER_L_CHANNEL_A, INPUT);
  pinMode(PIN_ENCODER_L_CHANNEL_B, INPUT);

  pinMode(PIN_ENCODER_R_CHANNEL_A, INPUT);
  pinMode(PIN_ENCODER_R_CHANNEL_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANNEL_A), count_time_left_channel_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_CHANNEL_B), count_time_left_channel_B, FALLING);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANNEL_A), count_time_right_channel_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_CHANNEL_B), count_time_right_channel_B, FALLING);
  previousTime_right = millis();
  previousTime_left = millis();
  
  // Configuração do TIMER1
  TCCR2A = 0b00100000;                //confira timer para operação normal
  TCCR2B = 0b00000011;                //limpa registrador
  
 
  OCR2A = 249;            // carrega registrador de comparação: 16MHz/1024/100.16Hz = 156 = 0X9c
  TIMSK2 = 0b00000010;

  
  //analogWrite(PIN_PWM_MOTOR_LIDAR, 240);
  //delay(1000);
}

void loop() {


 
      if (rpm_right_roda < 250) {
        output_right = compute_PID_right(rpm_right_roda);
      }
      if (rpm_left_roda < 250 ) {
        output_left = compute_PID_left(rpm_left_roda);
      }
      set_speed_motor(output_left, output_right);

      /*
      error_lidar = 230 -rpm_lidar ;
      out_lidar = error_lidar * Kp_lidar;
      if(out_lidar > 255){
        out_lidar = 255;
      }
      if(rpm_lidar < 220){
        out_lidar = 240;
      }
      analogWrite(PIN_PWM_MOTOR_LIDAR,out_lidar);
      */
      //Serial.println(String(error_lidar) + "," + String(out_lidar));
      //write_msg_serial();

      //Serial.print(set_rpm_left);
      //Serial.print(",");
      //Serial.println(set_rpm_right);

}

ISR(TIMER2_COMPA_vect)          // interrupção por igualdade de comparação no TIMER1
{
  write_msg_serial();
}

void write_msg_serial(){
  msg_serial_motor_rpms[1] = abs(rpm_left_roda);
  msg_serial_motor_rpms[2] = abs(rpm_right_roda);
  msg_serial_motor_rpms[3] = clock_wize_left*255;
  msg_serial_motor_rpms[4] = clock_wize_right*255;
  for(int i=0; i<5;i++)
      Serial.write(msg_serial_motor_rpms[i]); 
  //Serial.print(msg_serial_motor_rpms[1]);
  //Serial.print(",");
  //Serial.println(msg_serial_motor_rpms[2]);
}

double compute_PID_left(double inp) {
  currentTime_left = millis();                //get current time
  elapsedTime_left = (double)(currentTime_left - previousTime_left);        //compute time elapsed from previous computation

  error_left = set_rpm_left - inp;                                // determine error
  cumError_left += error_left * elapsedTime_left;                        // compute integral

  double out = kp_left * error_left + ki_left * cumError_left; // + kd*rateError;                //PID output
  out = (abs(error_left)==0.0 and abs(set_rpm_left) == 0.0)? 0.0 : out; 

  previousTime_left = currentTime_left;                        //remember current time

  return out;                                        //have function return the PID output
}

double compute_PID_right(double inp) {
  currentTime_right = millis();                //get current time
  elapsedTime_right = (double)(currentTime_right - previousTime_right);        //compute time elapsed from previous computation

  error_right = set_rpm_right - inp;                                // determine error
  cumError_right += error_right * elapsedTime_right;                        // compute integral

  double out = kp_right * error_right + ki_right * cumError_right; // + kd*rateError;                //PID output

  out = (abs(error_right)==0.0 and abs(set_rpm_right) == 0.0)? 0.0 : out; 
  previousTime_right = currentTime_right;                        //remember current time

  return out;                                        //have function return the PID output
}


void set_speed_motor(int motor_speed_L, int motor_speed_R) {

  if (motor_speed_R >= 0)
    write_motor(PIN_MOTOR_R1, PIN_MOTOR_R2);
  else
    write_motor(PIN_MOTOR_R2, PIN_MOTOR_R1);

  if (motor_speed_L >= 0)
    write_motor(PIN_MOTOR_L1, PIN_MOTOR_L2);
  else
    write_motor(PIN_MOTOR_L2, PIN_MOTOR_L1);

  if (motor_speed_R > 255 || motor_speed_R < -255)
    motor_speed_R = 255;
  if (motor_speed_L > 255 || motor_speed_L < -255)
    motor_speed_L = 255;
  //Serial.print(motor_speed_R);
  //Serial.print(" | ");
 
  //Serial.println(motor_speed_L);
  analogWrite(PIN_PWM_MOTOR_R, abs(motor_speed_R));
  analogWrite(PIN_PWM_MOTOR_L, abs(motor_speed_L));
}

void write_motor(int pin_motor_A, int pin_motor_B) {
  digitalWrite(pin_motor_A, HIGH);
  digitalWrite(pin_motor_B, LOW);
}

void decode_msg_rpm(char *msg_buffer_rpm_csv) {
  for (int i = 0; i < SIZE_MSG_01; i++) {
    msg_01[i] = msg_buffer_rpm[BEGIN_MSG_01 + i];
    msg_02[i] = msg_buffer_rpm[BEGIN_MSG_02 + i];
    //msg_03[i] = msg_buffer_rpm[BEGIN_MSG_03 + i];
  }

  //Serial.println(msg_buffer_rpm);
  set_rpm_left = atoi(msg_01);
  set_rpm_right = atoi(msg_02);
  //rpm_lidar = atoi(msg_03);
  //Serial.println(String(set_rpm_left) + "," + String(set_rpm_right) + "," + String(rpm_lidar));
}


void serialEvent() {
  bytes_received = Serial.readBytesUntil('\n', msg_buffer_rpm, SIZE_BUFFER_RPM);
  if (bytes_received > 0){
    decode_msg_rpm(msg_buffer_rpm);
    //Serial.println(msg_buffer_rpm);
  }
}

void count_time_left_channel_A() {
  time_left_channel_A = micros();
  
  if (digitalRead(PIN_ENCODER_L_CHANNEL_B)) {
    clock_wize_left = 1;
  } else {
    clock_wize_left = 0;
    dt_left = abs(time_left_channel_A - time_left_channel_B);
    rpm_left = ((1.0 / (6.0 * dt_left)) * 1e6) * 60 * pow(-1, clock_wize_left);
    if(dt_left > TIMEOUT_ENCODERS){
      rpm_left = 0;
    }
    rpm_left_roda = rpm_left / 75.0;

    last_rpm_left = rpm_left_roda;

  }
}

void count_time_left_channel_B() {
  time_left_channel_B = micros();

  if (clock_wize_left) {
    dt_left = abs(time_left_channel_B - time_left_channel_A);

    rpm_left = ((1.0 / (6.0 * dt_left)) * 1e6) * 60 * pow(-1, clock_wize_left);
    if(dt_left > TIMEOUT_ENCODERS){
      rpm_left = 0;  
    }
    rpm_left_roda = rpm_left / 75.0;

    last_rpm_left = rpm_left_roda;

  }
}

void count_time_right_channel_A() {
  time_right_channel_A = micros();

  if (digitalRead(PIN_ENCODER_R_CHANNEL_B)) {
    clock_wize_right = 1;
  } else {
    clock_wize_right = 0;
    dt_right = abs(time_right_channel_A - time_right_channel_B);
    rpm_right = ((1.0 / (6.0 * dt_right)) * 1e6) * 60 * pow(-1, clock_wize_right);
    if(dt_right > TIMEOUT_ENCODERS){
      rpm_right = 0;
    }
    rpm_right_roda = rpm_right / 75.0;

    last_rpm_right = rpm_right_roda;
  }
}

void count_time_right_channel_B() {
  time_right_channel_B = micros();

  if (clock_wize_right) {
    dt_right = abs(time_right_channel_B - time_right_channel_A);
    rpm_right = ((1.0 / (6.0 * dt_right)) * 1e6) * 60  * pow(-1, clock_wize_right);;
    
    if(dt_right > TIMEOUT_ENCODERS){
      rpm_right = 0;
    }
    rpm_right_roda = rpm_right / 75.0;

    last_rpm_right = rpm_right_roda;
  }
}
