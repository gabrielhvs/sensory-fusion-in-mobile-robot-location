#include <ros.h>
#include <geometry_msgs/Twist.h>

#define PIN_MOTOR_L1 13
#define PIN_MOTOR_L2 12
#define PIN_PWM_MOTOR_L 11

#define PIN_MOTOR_D1 8
#define PIN_MOTOR_D2 9
#define PIN_PWM_MOTOR_D 10


//Configure ROS
ros::NodeHandle nh;
geometry_msgs::Twist msg;

int linearVelocityX = 0;
int angularVelocityZ = 0;

//Create a message callback that updates the motor speeds
void messageCb(const geometry_msgs::Twist& cmd_vel)
{

  //Mecanum drive forward kinematics
    linearVelocityX = cmd_vel.linear.x;
    angularVelocityZ = cmd_vel.angular.z;
    
    
    if(angularVelocityZ > 0){
      setSpeedMotor(-linearVelocityX,linearVelocityX);
    }else if(angularVelocityZ < 0){
      setSpeedMotor(linearVelocityX,-linearVelocityX);
    }else{
      setSpeedMotor(linearVelocityX,linearVelocityX);
    }
}

//Subscribe to the Twist message
ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", &messageCb);


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_MOTOR_L1,OUTPUT);
  pinMode(PIN_MOTOR_L2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_L,OUTPUT);
  
  pinMode(PIN_MOTOR_D1,OUTPUT);
  pinMode(PIN_MOTOR_D2,OUTPUT);
  pinMode(PIN_PWM_MOTOR_D,OUTPUT);

  setSpeedMotor(0,0);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(10);
}

void setSpeedMotor(int motor_speed_L, int motor_speed_D){

  if(motor_speed_D >= 0)
    writeMotor(PIN_MOTOR_D1,PIN_MOTOR_D2);
  else
    writeMotor(PIN_MOTOR_D2,PIN_MOTOR_D1);

  if(motor_speed_L >= 0)
    writeMotor(PIN_MOTOR_L1,PIN_MOTOR_L2);
  else
    writeMotor(PIN_MOTOR_L2,PIN_MOTOR_L1);
  
  if(motor_speed_D > 255 || motor_speed_D < -255)
    motor_speed_D = 255;
  if(motor_speed_L > 255 || motor_speed_L < -255)
    motor_speed_L = 255;
    
  analogWrite(PIN_PWM_MOTOR_D,abs(motor_speed_D));
  analogWrite(PIN_PWM_MOTOR_L,abs(motor_speed_L));
}

void writeMotor(int pin_motor_A, int pin_motor_B){
  digitalWrite(pin_motor_A, HIGH);
  digitalWrite(pin_motor_B, LOW);
}
