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
        #self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.rate = rospy.Rate(10)
    
    def initVariables(self):
        self.state_led = '0'
        self.v = '0'
        self.w = '0'
        self.msg_csv = '0'

        self.rpm_wheel_left = 0
        self.rpm_wheel_right = 0

        self.rpm_scan = 0


    def initInstances(self):
        self.scan_msg = LaserScan()
        #self.rpm = Int16()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        #self.pose = self.rospy.Subscriber('/state_led', Int16, self.set_state_led, queue_size=10)
        #self.pose = self.rospy.Subscriber('/cmd_vel', Twist, self.set_cmd_vel, queue_size=10)
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

    def msg_serial(self):
        self.msg_csv = str(self.rpm_scan) + '\n'
        #print("MSG: ",self.msg_csv)
        self.serialArduino.write(self.msg_csv.encode())
    
    def read_msg_serial(self):
        data = self.serialArduino.read(2)
        #byte = int.from_bytes(data,byteorder='big')
        self.rpm_wheel_left = int(data[0])
        self.rpm_wheel_right = int(data[1])


        
if __name__ == '__main__':
        try:
            robot = Control_Ararajuba()
            while not rospy.is_shutdown():

                #print(robot.msg_csv)
                robot.read_msg_serial()
                #robot.read_rpm_wheel()
                #robot.read_serial()
                robot.write_wheel()
                #print(robot.rpm_wheel_left, ",", robot.rpm_wheel_right)
                #rospy.spin()
        except rospy.ROSInterruptException: pass