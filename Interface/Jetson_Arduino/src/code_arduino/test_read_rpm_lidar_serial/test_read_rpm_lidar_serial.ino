#define PIN_PWM_LIDAR 3

int rpm_scan = 0;
char msg_buffer[4];
bool bytes_rec;

int set_rpm_lidar = 220;
uint8_t motor_pwm_lidar = 240;
double Kp_lidar = 2;
double Ki_lidar = 0;
int error_lidar = 0;
int tol = 5;

unsigned long last_time_lidar = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  pinMode(PIN_PWM_LIDAR,OUTPUT);
  analogWrite(PIN_PWM_LIDAR, motor_pwm_lidar);
  
  last_time_lidar = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("aqui");
  //dt_lidar = millis() - last_time_lidar;
  //last_time_lidar = millis();
  if(rpm_scan-tol < set_rpm_lidar){
 
    motor_pwm_lidar++;
  }else if(rpm_scan+tol > set_rpm_lidar){
    motor_pwm_lidar--;
  }
  
  if(motor_pwm_lidar < 130 || motor_pwm_lidar > 255)
    motor_pwm_lidar = 240;
  
  analogWrite(PIN_PWM_LIDAR, motor_pwm_lidar);
  
  Serial.println(String(rpm_scan) + "|" + String(set_rpm_lidar) + "|" + String(motor_pwm_lidar));
 
}


void serialEvent(){
  bytes_rec = Serial.readBytesUntil('\n',msg_buffer,3);
  if(bytes_rec){
    rpm_scan = atoi(msg_buffer); 
  }
}
