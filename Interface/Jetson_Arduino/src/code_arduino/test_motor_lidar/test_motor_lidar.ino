#define PIN_MOTOR_LIDAR 2
#define PIN_BOTAO 13

// --- Variables ---
int data_byte = 0;         // incoming serial byte
unsigned char data_status = 0;
unsigned char data_4deg_index = 0;
unsigned char data_index = 0;
unsigned char speed_RPH_high_byte = 0; //
unsigned char speed_RPH_low_byte = 0;

double set_rpm_lidar = 0;
double rpm_lidar = 0;
double Kp_lidar = 1;

uint8_t motor_pwm_lidar = 0;


long now = 0;

size_t bytes_received;
char msg_buffer[3 + 1];


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);  // USB serial
  Serial3.begin(115200);  // XV-11 LDS

  pinMode(PIN_MOTOR_LIDAR, OUTPUT);
  pinMode(PIN_BOTAO, INPUT_PULLUP);


  //analogWrite(PIN_MOTOR_LIDAR, 255);
}

void loop() {
  /*
    motor_pwm_lidar += PID(set_rpm_lidar, rpm_lidar, Kp_lidar);
    analogWrite(PIN_MOTOR_LIDAR, motor_pwm_lidar);
  */
  /*
  if (!digitalRead(13)) {
    analogWrite(PIN_MOTOR_LIDAR, 255);
    now = millis();
    Serial.print(now);
    Serial.print(',');
    Serial.println(rpm_lidar);

  } else {

    Serial.println("Antes");
  }
  */
  //Serial.println(" ");
  
}


void serialEvent() {
  Serial.readBytesUntil('\n', msg_buffer, 3);
  motor_pwm_lidar = atoi(msg_buffer);
}

void serialEvent3() {
  data_byte = Serial3.read();
  decodeData();
  //Serial.write(data_byte);
}


void decodeData() {
  switch (data_status) {
    case 0: // no header
      if (data_byte == 0xFA)
      {
        data_status = 1;
        data_index = 1;
      }
      break;
    case 1: // Find 2nd FA
      if (data_index == 22) {
        if (data_byte == 0xFA)
        {
          data_status = 2;
          data_index = 1;
        }
        else // if not FA search again
          data_status = 0;
      }
      else {
        data_index++;
      }
      break;
    case 2: // Read data out

      if (data_index == 22) {
        if (data_byte == 0xFA)
        {
          data_index = 1;
        }
        else // if not FA search again
          data_status = 0;
      }
      else {
        readData();
        data_index++;
      }
      break;
  }

}
void readData() {
  switch (data_index) {
    case 1: // 4 degree index
      data_4deg_index = data_byte - 0xA0;
      break;
    case 2: // Speed in RPH low byte
      speed_RPH_low_byte = data_byte;
      break;
    case 3: // Speed in RPH high byte
      speed_RPH_high_byte = data_byte;
      rpm_lidar = ((speed_RPH_high_byte << 8) | speed_RPH_low_byte) / 64;
      //Serial.print(millis());
      //Serial.print(',');
      Serial.println(rpm_lidar);
      break;
    default: // others do checksum
      break;
  }
}
