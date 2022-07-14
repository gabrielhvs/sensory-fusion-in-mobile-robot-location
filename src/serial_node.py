#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg  import Int16
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan


class Control_Ararajuba:
    
    def __init__(self):

        self.serialArduino = serial.Serial(port='/dev/ttyUSB0',baudrate=115200)

        self.rospy=rospy
        self.rospy.init_node('serial_node', anonymous=True)
        self.initInstances()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.rate = rospy.Rate(10)
    
    def initVariables(self):
        self.state_led = '0'
        self.v = '0'
        self.w = '0'
        self.msg_csv = '0'

        self.distances = [0] * 360
        self.intensities = [0] * 360
        self.rpm = 0
        self.packet = [0] * 22

        self.startReading = False
        self.packetNumber = 0
        self.speed = 0
        self.packetIndex = 0
        self.distanceIndex = 0

    def initInstances(self):
        self.scan_msg = LaserScan()
        #self.rpm = Int16()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        self.pose = self.rospy.Subscriber('/state_led', Int16, self.set_state_led, queue_size=10)
        self.pose = self.rospy.Subscriber('/cmd_vel', Twist, self.set_cmd_vel, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=50)
        self.rpm_publisher = rospy.Publisher('/rpm_scan', Int16, queue_size=10)
        self.rate = rospy.Rate(1.0)

    '''
    def scan_write(self):
        current_time = rospy.Time.now()

        self.scan.header.stamp = current_time
        self.scan.header.frame_id = 'neato_laser'
        self.scan.angle_min = 0
        self.scan.angle_max = 2.0 * 3.14159
        self.scan.angle_increment = (2.0 * 3.14159) / 360.0
        self.scan.time_increment = (1.0 / 6) / (360)
        self.scan.range_min = 0.06
        self.scan.range_max = 5.0

        self.scan.ranges = []
        self.scan.intensities = []

        self.scan_publisher.publish(self.scan)

        self.rate.sleep()
    '''
    def scan_write2(self):

        current_time = rospy.Time.now()

        scan = LaserScan()

        scan.header.stamp = current_time
        scan.header.frame_id = 'neato_laser'
        scan.angle_min = 0
        scan.angle_max = 2.0 * 3.14159
        scan.angle_increment = (2.0 * 3.14159) / 360.0
        scan.time_increment = (1.0 / 6) / (360)
        scan.range_min = 0.06
        scan.range_max = 5.0

        #print(self.distances)
        scan.ranges = self.distances
        scan.intensities = self.intensities

        self.scan_publisher.publish(scan) 
        self.rpm_publisher.publish(int(self.rpm)) 


    def set_state_led(self,msg):
        self.state_led = str(msg.data)
        self.msg_serial()

    def set_cmd_vel(self, msg):
        self.v = int(float(str(msg.linear.x)))
        self.w = int(float(str(msg.angular.z)))
        if(self.v < 10):
            self.v = "00" + str(self.v)
        elif(self.v < 100):
            self.v = "0" + str(self.v)
        else:
            self.v = str(self.v)

        if(self.w < 10):
            self.w = "00" + str(self.w)
        elif(self.w < 100):
            self.w = "0" + str(self.w)
        else:
            self.w = str(self.w)
        print("PWM R: ",self.v," | PWM L: ",self.w)
        self.msg_csv = str(self.v) + "," + str(self.w)
        print("MSG: ",self.msg_csv)
        self.serialArduino.write(self.msg_csv.encode())
    
    def msg_serial(self):
        self.msg_csv = self.v + "," + self.w
        print("MSG: ",self.msg_csv)
        self.serialArduino.write(self.msg_csv.encode())

    def read_serial(self):
        data = self.serialArduino.read()
        byte = int.from_bytes(data,byteorder='big')
        if self.startReading:
            if self.packetIndex == 21:
                self.startReading = False
                self.processEndOfPacket()
                # let the calling function know we're done
                return self.distanceIndex >= 359
            else:
                self.packet[self.packetIndex] = byte
                self.packetIndex += 1
                return False

        if not self.startReading and byte == 0xFA:
            self.startReading = True
            self.packetIndex = 0
            self.packet[self.packetIndex] = byte
            self.packetIndex += 1
            return False

    def processEndOfPacket(self):
        newPacketNumber = self.packet[1]
        #print(newPacketNumber,newPacketNumber - self.packetNumber)
        if (newPacketNumber - self.packetNumber) == 1:
            for i in range(22):
                #print(i)
                if i == 2:
                    self.rpm = (self.packet[3] << 8 | self.packet[2]) / 64
                    #self.rpm = (self.packet[2])
                elif self.isDataIndex(i):
                    self.distanceIndex = int((newPacketNumber - 0xA0) * 4 + i/4 - 1)
                    if self.distanceIndex >= 0 and self.distanceIndex < 360:
                        if (self.packet[i+1] & 0x80) >> 7:
                            self.distances[self.distanceIndex] = 0
                        else:
                            d = (self.packet[i] | ((self.packet[i+1] & 0x3F) << 8))
                            i = (self.packet[i+3] << 8) | self.packet[i+2]
                            # cap between limits
                            if d < 100 or d > 6000:
                                d = 0

                            # finally, store valid data
                            self.distances[self.distanceIndex] = d/1000
                            self.intensities[self.distanceIndex] = i
                else:
                    #checksum, packet num, and start bit go here
                    pass

        self.packetNumber = newPacketNumber

        if self.packetNumber >= 0xF9:
            self.packetNumber = 0x9F


    def isDataIndex(self, index):
        return index == 4 or index == 8 or index == 12 or index == 16

    def checksum(self, data):
        """Compute and return the checksum as an int."""
        # group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append(data[2*t] + (data[2*t+1]<<8))

            # compute the checksum on 32 bits
            chk32 = 0
            for d in data_list:
                chk32 = (chk32 << 1) + d

            # return a value wrapped around on 15bits
            # and truncated to still fit into 15 bits
            # then wrap around to fit into 15 bits
            checksum = (chk32 & 0x7FFF) + (chk32 >> 15)

            # truncate to 15 bits
            checksum = checksum & 0x7FFF

            return int(checksum)

    def getLatestPacket(self):
        return self.packet

    def getLatestDistance(self):
        return self.distances

    def getLatestIntensity(self):
        return self.intensities

    
        
if __name__ == '__main__':
        try:
            robot = Control_Ararajuba()
            while not rospy.is_shutdown():
                robot.read_serial()
                #print(robot.getLatestDistance())
                robot.scan_write2()
                #print(robot.getLatestIntensity())
                #print(robot.getLatestDistance())
                #print(robot.getLatestPacket())
                #print(robot.rpm)
                #rospy.spin()
        except rospy.ROSInterruptException: pass