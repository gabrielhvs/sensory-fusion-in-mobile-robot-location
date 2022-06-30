#include <TimerThree.h>                   // used for ultrasonic PWM motor control
#include <PID.h>

int motor_pwm_pin = 11;            // pin connected to mosfet for motor speed control
double rpm_setpoint = 200;          // desired RPM (uses double to be compatible with PID library)
double rpm_min = 200;
double rpm_max = 300;
double pwm_max = 255;              // max analog value.  probably never needs to change from 1023
double pwm_min = 150;              // min analog pulse value to spin the motor
int sample_time = 20;             // how often to calculate the PID values

// PID tuning values
double Kp = 2.0;
double Ki = 1.0;
double Kd = 0.0;

double pwm_val = 500;          // start with ~50% power
double pwm_last;
double motor_rpm;
unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = 200;
unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
unsigned long curMillis;
unsigned long lastMillis = millis();

PID rpmPID(&motor_rpm, &pwm_val, &rpm_setpoint, Kp, Ki, Kd, DIRECT);

const unsigned char COMMAND = 0xFA;        // Start of new packet
const int INDEX_LO = 0xA0;                 // lowest index value
const int INDEX_HI = 0xF9;                 // highest index value

const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int OFFSET_TO_START = 0;
const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet

int Packet[PACKET_LENGTH];                 // an input packet
int ixPacket = 0;                          // index into 'Packet' array
const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
int eState = eState_Find_COMMAND;

uint8_t inByte = 0;  // incoming serial byte
uint8_t motor_rph_high_byte = 0;
uint8_t motor_rph_low_byte = 0;
uint16_t motor_rph = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor_pwm_pin, OUTPUT);
  Serial2.begin(115200);                    // USB serial

  Timer3.initialize(30);                           // set PWM frequency to 32.768kHz

  rpmPID.SetOutputLimits( pwm_min,  pwm_max);
  rpmPID.SetSampleTime( sample_time);
  rpmPID.SetTunings( Kp,  Ki,  Kd);
  rpmPID.SetMode(AUTOMATIC);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial1.available() > 0) {                  // read byte from LIDAR and relay to USB
    inByte = Serial1.read();                      // get incoming byte:
    
    Serial.write(inByte);                 // relay

    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
        //if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
          processSpeed();                             // process the speed
        //}
      }
    }
  }
  rpmPID.Compute();
  if (pwm_val != pwm_last) {
    Timer3.pwm(motor_pwm_pin, pwm_val);  // replacement for analogWrite()
    pwm_last = pwm_val;
  }
  motorCheck();
}

void processSpeed() {
  motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}

void motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval) {
    if ((motor_rpm < rpm_min or motor_rpm > rpm_max) and pwm_val > 1000) {
      rpm_err++;
    }
    else {
      rpm_err = 0;
    }
    if (rpm_err > rpm_err_thresh) {
       Timer3.pwm(motor_pwm_pin, 0);
    }
    motor_check_timer = millis();
  }
}
