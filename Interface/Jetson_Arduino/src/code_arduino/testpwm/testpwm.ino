#define pinPWM 9 
#define PWM 155
void setup() {
  // put your setup code here, to run once:
  pinMode(pinPWM,OUTPUT);
  Serial.begin(9600);
  analogWrite(pinPWM,PWM);
}

void loop() {
 
}
