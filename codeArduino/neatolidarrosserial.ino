#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>

#define N_ANGLES 360                // # of angles (0..359)
#define PACKET_SIZE 22
#define PACKET_FOR_REVOLUTION 90
#define PACKET_FULL_LENGTH 1980
#define COMMAND_INIT_PACKET 0xFA

uint8_t Packet[PACKET_FULL_LENGTH] = {0};
uint16_t ixPacket;
uint16_t index;

#define eState_Find_COMMAND 0                        // 1st state: find 0xFA (COMMAND) in input stream
#define eState_Build_Packet 1  // 2nd state: build the packet
bool eState = eState_Find_COMMAND;

uint8_t inByte;
uint8_t good_sets = 0;
uint32_t motor_speed = 0;
uint32_t rpms = 0;

ros::NodeHandle  nh;

sensor_msgs::LaserScan neato_LDS_msg;
ros::Publisher laser_pub("scan_arduino", &neato_LDS_msg);

//int16_t ranges = (int16_t*) malloc(sizeof(int16_t)*360);
//int16_t intensities = (int16_t*) malloc(sizeof(int16_t)*360);

void setup()
{
  Serial2.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(laser_pub);

  neato_LDS_msg.header.frame_id =  "/neato_laser";
  neato_LDS_msg.angle_min = 0.0;
  neato_LDS_msg.angle_max = 2.0 * M_PI;
  neato_LDS_msg.angle_increment = (2.0 * M_PI / 360.0);
  neato_LDS_msg.range_min = 0.06;
  neato_LDS_msg.range_max = 5.0;

  float *ranges = new float[N_ANGLES] {2};
  float *intensities = new float[N_ANGLES] {8};

  neato_LDS_msg.ranges_length = N_ANGLES;
  neato_LDS_msg.intensities_length = N_ANGLES;
  neato_LDS_msg.ranges = ranges;
  neato_LDS_msg.intensities = intensities;


  //for (ixPacket = PACKET_FULL_LENGTH - 1; ixPacket >= 0; ixPacket--)  // Initialize
  //Packet[ixPacket] = 0;

}

void loop()
{

  poll(neato_LDS_msg.ranges, neato_LDS_msg.intensities );
  neato_LDS_msg.header.stamp = nh.now();
  laser_pub.publish(&neato_LDS_msg);
  
  nh.spinOnce();

}


void poll(float *ranges, float *intensities) {

  if (Serial2.available()) {
    uint8_t inByte = Serial2.read();
    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND_INIT_PACKET) {
        eState = 1;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
        if (inByte == COMMAND_INIT_PACKET) {
          eState = 1;                                 // switch to 'build a packet' state
          Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
        }
      }
      else {                                            // eState == eState_Build_Packet
        Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
        if (ixPacket == PACKET_FULL_LENGTH) {
          //read data in sets of 4
          for (uint16_t i = 0; i < PACKET_FULL_LENGTH; i = i + 22) {
            if (Packet[i] == 0xFA && Packet[i + 1] == (0xA0 + i / 22)) { //&& CRC check
              good_sets++;
              //motor_speed += (Packet[i + 3] << 8) + Packet[i + 2]; //accumulate count for avg. time increment
              rpms = (Packet[i + 3] << 8 | Packet[i + 2]) / 64;
              //intensities[359] = i;
              
              for (uint16_t j = i + 4; j < i + 20; j = j + 4) {
                index = (4 * i) / 22 + (j - 4 - i) / 4;
                // Four bytes per reading
                
                
                uint8_t byte0 = Packet[j];
                uint8_t byte1 = Packet[j + 1];
                uint8_t byte2 = Packet[j + 2];
                uint8_t byte3 = Packet[j + 3];
                // First two bits of byte1 are status flags
                // uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
                // uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
                // Remaining bits are the range in mm
                uint16_t range = ((byte1 & 0x3F) << 8) + byte0;
                // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                uint16_t intensity = (byte3 << 8) + byte2;

                ranges[index] = range / 1000.0;
                intensities[index] = intensity;
                
              }
            }
          }
          for (ixPacket = PACKET_FULL_LENGTH - 1; ixPacket >= 0; ixPacket--)  // Initialize
            Packet[ixPacket] = 0;
          ixPacket = 0;
          eState = eState_Find_COMMAND;
        }
      }
    }
  }
}
