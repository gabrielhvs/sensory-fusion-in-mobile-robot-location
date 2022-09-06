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
#define MODO_INPUT_PWM 1
#define SIZE_BUFFER_PWM 9
#define SIZE_BUFFER_RPM 5

#define BEGIN_MSG_01 0
#define BEGIN_MSG_02 5
#define SIZE_MSG_01 4
#define SIZE_MSG_01 4

// =====================


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
char msg_buffer[SIZE_BUFFER_PWM + 1];
char msg_01[5];
char msg_02[5];


float set_rpm_left = 180;
float error_rpm_left = 0;
float Kp_left = 3.7445;
float Ki_left = 0.0225;
double output_left = 0;
long cumError_left = 0;
long previousTime_left = 0;

float set_rpm_right = 100;
float error_rpm_right = 0;
float Kp_right = 4.879;
float Ki_right = 0.0336;
double output_right = 0;
long cumError_right = 0;
long previousTime_right = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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

}

void loop() {
  //Serial.print("RPM Right: ");


  //Serial.print(rpm_right_roda);
  //Serial.print(",");
  //Serial.println(rpm_left_roda);

  bytes_received = Serial.readBytesUntil('\n', msg_buffer, SIZE_BUFFER_PWM);
  if (bytes_received > 0) {
    //Serial.println(msg_buffer);
    decode_msg_csv(msg_buffer);
    set_speed_motor(motor_pwm_left, motor_pwm_right);
  }


  //output_left = computePID(set_rpm_left,rpm_left_roda, Kp_left, Ki_left, &cumError_left, &previousTime_right);
  //output_right = computePID(set_rpm_right,rpm_right_roda, Kp_right, Ki_right, &cumError_right, &previousTime_right);
  output_left = -100;
  output_right = -100;
   set_speed_motor( output_left, output_right);

  Serial.print(rpm_left_roda);
  Serial.print(",");
  Serial.println(rpm_right_roda);






}

double computePID(double set_point, double feedback, int Kp, int Ki, long *cumError, long *previousTime){
        double elapsedTime = (double)(millis() - (*previousTime));        //compute time elapsed from previous computation
       
        float error = set_point - feedback;  
        Serial.println(error);
        *cumError = (*cumError) + (error * elapsedTime);                        // compute integral
 
        double out = (Kp*error) + (Ki*(*cumError));// + kd*rateError;                //PID output
                                
        *previousTime = millis();                        //remember current time
 
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
  //Serial.print("\t");
  //Serial.println(motor_speed_R);
  //Serial.print(" | ");
  //Serial.println(motor_speed_L);
  analogWrite(PIN_PWM_MOTOR_R, abs(motor_speed_R));
  analogWrite(PIN_PWM_MOTOR_L, abs(motor_speed_L));
}

void write_motor(int pin_motor_A, int pin_motor_B) {
  digitalWrite(pin_motor_A, HIGH);
  digitalWrite(pin_motor_B, LOW);
}

void decode_msg_csv(char *msg_buffer_csv) {
  for (int i = 0; i < SIZE_MSG_01; i++) {
    msg_01[i] = msg_buffer[BEGIN_MSG_01 + i];
    msg_02[i] = msg_buffer[BEGIN_MSG_02 + i];
  }

  motor_pwm_left = atoi(msg_01);
  motor_pwm_right = atoi(msg_02);
}

void decode_msg(char *msg_buffer_csv) {

  motor_pwm_left = atoi(msg_buffer_csv);
  motor_pwm_right = atoi(msg_buffer_csv);

  //Serial.println(msg_buffer_csv);
  //Serial.print(set_rpm_left);
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
