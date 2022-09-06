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

// --- Serial ---


#define MODO_INPUT 1
#define SIZE_BUFFER 9
#define SIZE_BUFFER_PWM 9
#define SIZE_BUFFER_RPM 7
#define SIZE_BUFFER_CMD_VEL 9
#define BEGIN_MSG_01 0
#define BEGIN_MSG_02 5
#define SIZE_MSG_01 4
#define SIZE_MSG_01 4

// =====================


// --- Variables ---
int data_byte = 0;         // incoming serial byte
unsigned char data_status = 0;
unsigned char data_4deg_index = 0;
unsigned char data_index = 0;
unsigned char speed_RPH_high_byte = 0; //
unsigned char speed_RPH_low_byte = 0;



int rpm_lidar = 0;

float rpm_left = 0;
int rpm_left_roda = 0;
int clock_wize_left = 1;
int motor_pwm_left = 0;

float rpm_right = 0;
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
char msg_buffer[SIZE_BUFFER + 1];
char msg_buffer_pwm[SIZE_BUFFER_PWM + 1];
char msg_buffer_rpm[SIZE_BUFFER_RPM + 1];
char msg_buffer_cmd_vel[SIZE_BUFFER_CMD_VEL + 1];
char msg_01[5];
char msg_02[5];

double kp_left = 3.740;
double ki_left = 0.0227;
double kd_left = 0;

unsigned long currentTime_left, previousTime_left;
double elapsedTime_left;
double error_left;
double lastError_left;
int set_rpm_left = 100;
double input_left, output_left;
double cumError_left, rateError_left;

double kp_right = 4.87;
double ki_right = 0.0336;
double kd_right = 0;

unsigned long currentTime_right, previousTime_right;
double elapsedTime_right;
double error_right;
double lastError_right;
int set_rpm_right = 100;
double input_right, output_right;
double cumError_right, rateError_right;
//float Ki = 10.5562;
//float Kd = 0.00046;


byte msg_serial_motor_rpms[2] = {0};

void setup() {
  Serial.begin(115200);  // USB serial
  Serial3.begin(115200);  // XV-11 LDS data

  pinMode(13, INPUT_PULLUP);

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
}

void loop() {


 
      if (rpm_right_roda < 250) {
        output_right = compute_PID_right(rpm_right_roda);
      }
      if (rpm_left_roda < 250 ) {
        output_left = compute_PID_left(rpm_left_roda);
      }
      set_speed_motor(output_left, output_right);
      write_msg_serial();

//      Serial.print(rpm_left_roda);
//      Serial.print(",");
//      Serial.println(rpm_right_roda);

}

void write_msg_serial(){
  msg_serial_motor_rpms[0] = rpm_left_roda;
  msg_serial_motor_rpms[1] = rpm_right_roda;
  Serial.write(msg_serial_motor_rpms,2); 
}

double compute_PID_left(double inp) {
  currentTime_left = millis();                //get current time
  elapsedTime_left = (double)(currentTime_left - previousTime_left);        //compute time elapsed from previous computation

  error_left = set_rpm_left - inp;                                // determine error
  cumError_left += error_left * elapsedTime_left;                        // compute integral

  double out = kp_left * error_left + ki_left * cumError_left; // + kd*rateError;                //PID output

  previousTime_left = currentTime_left;                        //remember current time

  return out;                                        //have function return the PID output
}

double compute_PID_right(double inp) {
  currentTime_right = millis();                //get current time
  elapsedTime_right = (double)(currentTime_right - previousTime_right);        //compute time elapsed from previous computation

  error_right = set_rpm_right - inp;                                // determine error
  cumError_right += error_right * elapsedTime_right;                        // compute integral

  double out = kp_right * error_right + ki_right * cumError_right; // + kd*rateError;                //PID output

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
  //  Serial.print(motor_speed_R);
  //  Serial.print(" | ");
  //  Serial.println(motor_speed_L);
  analogWrite(PIN_PWM_MOTOR_R, abs(motor_speed_R));
  analogWrite(PIN_PWM_MOTOR_L, abs(motor_speed_L));
}

void write_motor(int pin_motor_A, int pin_motor_B) {
  digitalWrite(pin_motor_A, HIGH);
  digitalWrite(pin_motor_B, LOW);
}

void decode_msg_pwm(char *msg_buffer_csv) {
  for (int i = 0; i < SIZE_MSG_01; i++) {
    msg_01[i] = msg_buffer[BEGIN_MSG_01 + i];
    msg_02[i] = msg_buffer[BEGIN_MSG_02 + i];
  }

  //Serial.println(msg_buffer);
  set_rpm_left = atoi(msg_01);
  set_rpm_right = atoi(msg_02);
  //Serial.println(String(set_rpm_left) + "," + String(set_rpm_right));
}

void decode_msg_rpm(char *msg_buffer_csv) {
  for (int i = 0; i < SIZE_MSG_01; i++) {
    msg_01[i] = msg_buffer[BEGIN_MSG_01 + i];
    msg_02[i] = msg_buffer[BEGIN_MSG_02 + i];
  }
  //Serial.println(msg_buffer);
  set_rpm_left = atoi(msg_01);
  set_rpm_right = atoi(msg_02);
  //Serial.println(String(set_rpm_left) + "," + String(set_rpm_right));

}



void serialEvent() {
  bytes_received = Serial.readBytesUntil('\n', msg_buffer, SIZE_BUFFER_PWM);
  if (bytes_received > 0)
    decode_msg_pwm(msg_buffer);

  //Serial.println(msg_buffer);
}

void count_time_left_channel_A() {
  time_left_channel_A = micros();

  if (digitalRead(PIN_ENCODER_L_CHANNEL_B)) {
    clock_wize_left = 1;
  } else {
    clock_wize_left = 0;
    dt_left = abs(time_left_channel_A - time_left_channel_B);
    rpm_left = ((1.0 / (6.0 * dt_left)) * 1e6) * 60 * pow(-1, clock_wize_left);
    rpm_left_roda = rpm_left / 75.0;

  }
}

void count_time_left_channel_B() {
  time_left_channel_B = micros();

  if (clock_wize_left) {
    dt_left = abs(time_left_channel_B - time_left_channel_A);

    rpm_left = ((1.0 / (6.0 * dt_left)) * 1e6) * 60 * pow(-1, clock_wize_left);
    rpm_left_roda = rpm_left / 75.0;

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
    rpm_right_roda = rpm_right / 75.0;
  }
}

void count_time_right_channel_B() {
  time_right_channel_B = micros();

  if (clock_wize_right) {
    dt_right = abs(time_right_channel_B - time_right_channel_A);
    rpm_right = ((1.0 / (6.0 * dt_right)) * 1e6) * 60  * pow(-1, clock_wize_right);;
    rpm_right_roda = rpm_right / 75.0;
  }
}
