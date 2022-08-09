#!/usr/bin/env python
from operator import index
import serial
import rospy
from std_msgs.msg  import Int16
from std_msgs.msg  import UInt16
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
import numpy as np

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

        self.R = (11.16/2)/100 #in meters
        self.L = (2*2.13+13.84+13.35)/100 #in meters

        self.set_rpm_wheel_left = 0
        self.set_rpm_wheel_right = 0

        self.rpm_wheel_left = 0
        self.rpm_wheel_right = 0

        self.rpm_scan = 0

        self.start_recive = False
        self.index = 0


    def initInstances(self):
        self.scan_msg = LaserScan()
        #self.rpm = Int16()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        #self.pose = self.rospy.Subscriber('/state_led', Int16, self.set_state_led, queue_size=10)
        self.rospy.Subscriber('/cmd_vel', Twist, self.set_cmd_vel, queue_size=10)
        self.rospy.Subscriber('/rpm_scan', Int16, self.set_rpm_scan, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        #self.scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=50)
        self.rpm_wheel_left_publisher = rospy.Publisher('/rpm_wheel_left', Int16, queue_size=10)
        self.rpm_wheel_right_publisher = rospy.Publisher('/rpm_wheel_right', Int16, queue_size=10)
        self.rate = rospy.Rate(10)

    def write_wheel(self):
        self.rpm_wheel_left_publisher.publish(int(self.rpm_wheel_left)) 
        self.rpm_wheel_right_publisher.publish(int(self.rpm_wheel_right)) 

    def set_rpm_scan(self,msg):
        self.rpm_scan = msg.data
        #print(self.rpm_scan)
        self.msg_serial()
    
    def set_cmd_vel(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        
        self.set_rpm_wheel_right = int((1/self.R)*( self.v + (self.w*self.L)))
        self.set_rpm_wheel_left = int((1/self.R)*( self.v - (self.w*self.L)))
        
        if(self.set_rpm_wheel_right < 30 and self.set_rpm_wheel_right > -30):
            self.set_rpm_wheel_right = 0
        if(self.set_rpm_wheel_left < 30 and self.set_rpm_wheel_left > -30):
            self.set_rpm_wheel_left = 0
        
        self.set_rpm_wheel_left = self.convert_format_cmd_vel(self.set_rpm_wheel_left)
        self.set_rpm_wheel_right = self.convert_format_cmd_vel(self.set_rpm_wheel_right)
        
        self.msg_csv = self.set_rpm_wheel_left + "," + self.set_rpm_wheel_right
        
        print("MSG: ",self.msg_csv)
        self.serialArduino.write(self.msg_csv.encode())

    def convert_format_cmd_vel(self,set_rpm):
        msg = list("+000")

        if(set_rpm < 0):
            msg[0]= '-'

        set_rpm = abs(set_rpm)
        if(set_rpm < 10):
            msg[3] = str(set_rpm)
        elif(set_rpm < 100):
            msg[2:] = str(set_rpm)
        elif(set_rpm < 1000): 
            msg[1:] = str(set_rpm)

        msg = ''.join(msg)
        return msg

    def msg_serial(self):
        self.msg_csv = str(self.rpm_scan) + '\n'
        print("MSG: ",self.msg_csv)
        self.serialArduino.write(self.msg_csv.encode())
    
    def read_msg_serial(self):
        data = self.serialArduino.read()
        byte = int.from_bytes(data,byteorder='big')
        #print(byte)
        if(byte == 253):
            self.start_recive = True
        else:
            if(self.start_recive):
                self.index+=1
                if self.index == 1:
                    self.rpm_wheel_left  = byte
                elif self.index == 2:
                    self.rpm_wheel_right = byte
                elif self.index == 3:
                    if(not byte):
                        self.rpm_wheel_left = self.rpm_wheel_left * -1
                elif self.index == 4:
                    if(not byte):
                        self.rpm_wheel_right = self.rpm_wheel_right * -1
                    self.start_recive = False
                    print("Pacote: ", self.rpm_wheel_left, self.rpm_wheel_right)
                    self.index = 0

        #self.dt_left = int((data[2]))*100+int(data[3])
        #self.dt_right = int((data[4]<<2))+int(data[5])
        #print(self.rpm_wheel_left)


        
if __name__ == '__main__':
        try:
            robot = Control_Ararajuba()
            while not rospy.is_shutdown():

                #print(robot.msg_csv)
                robot.read_msg_serial()
                #print(robot.v)
                #robot.read_rpm_wheel()
                #robot.read_serial()
                robot.write_wheel()
                #print(robot.rpm_wheel_left, ",", robot.rpm_wheel_right)
                #rospy.spin()
        except rospy.ROSInterruptException: pass