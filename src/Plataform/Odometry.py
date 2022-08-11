from cmath import pi
from numpy import *
import numpy as np
import matplotlib.pyplot as plt
#!/usr/bin/env python
import rospy
import time
from math import pi,cos, sin,tan, atan
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

class Calc_Odom:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('Odom')
        self.initInstances()
        self.initSubscribers()
        self.initPublishers()
        self.initvariables()
        self.rate = rospy.Rate(10)
    
    def initInstances(self):
        self.odom_msg = Odometry()
        self.dt = 1/1000
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        self.velR = self.rospy.Subscriber('rpm_wheel_right', Int16, self.get_velRight, queue_size=10)
        self.velL = self.rospy.Subscriber('rpm_wheel_left', Int16, self.get_velLeft, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(10)

    def initvariables(self):
        self.R = (11.16/2)/100 #in meters
        self.L = (2*2.13+13.84+13.35)/100 #in meters
        self.Theta = 0
        self.X = 0
        self.Y = 0
        
    def get_velRight(self, msg):

        self.velRight = (msg.data/60)*2*pi*self.R
        #self.velRight = msg.data
        
    def get_velLeft(self, msg):

        self.velLeft = (msg.data/60)*2*pi*self.R
        #self.velLeft = msg.data
    
    def get_distR(self):
        self.distanceR = self.velRight*self.dt
        return self.distanceR


    def get_distL(self):
        self.distanceL = self.velLeft*self.dt
        return self.distanceL
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def set_odom(self):
        
        #Get Dist.
        self.get_distL()
        self.get_distR()

        #Velocities Calc. 
        vl = (self.velRight+self.velLeft)/2
        w= (self.velRight-self.velLeft)/(2*self.L)
        
        #Pose Calc.
        distance_center = (self.distanceL + self.distanceR)/2
        dtheta = (self.distanceL - self.distanceR)/(2*self.L)

        self.X = self.X+distance_center*cos(self.Theta)
        self.Y = self.Y+distance_center*sin(self.Theta)
        self.Theta = self.Theta+dtheta
        
        
        [qx, qy, qz, qw] = self.get_quaternion_from_euler(0,0,self.Theta)
        print(180*dtheta/pi)
        now =rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0
        self.odom_msg.pose.pose.orientation.x = qx
        self.odom_msg.pose.pose.orientation.y = qy
        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.twist.twist.linear.x = vl
        self.odom_msg.twist.twist.angular.z = w
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.header.stamp.secs = now.secs
        self.odom_msg.header.stamp.nsecs = now.nsecs

        self.odom_publisher.publish(self.odom_msg)

        
        
        
if __name__ == '__main__':
        
        try:
            odom = Calc_Odom()
            time.sleep(0.1)
            while not rospy.is_shutdown():
                
               odom.set_odom() 

        except rospy.ROSInterruptException: pass
