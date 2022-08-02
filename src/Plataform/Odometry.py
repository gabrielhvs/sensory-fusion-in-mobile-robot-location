from cmath import pi
import scipy
from numpy import *
import numpy as np
import matplotlib.pyplot as plt
#!/usr/bin/env python
import rospy
import time
from math import pi,cos
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
        self.beforeR = rospy.get_rostime()
        self.beforeL = rospy.get_rostime()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        self.velR = self.rospy.Subscriber('velRight', Int16, self.get_velRight, queue_size=10)
        self.velL = self.rospy.Subscriber('velLeft', Int16, self.get_velLeft, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(10)

    def initvariables(self):
        self.R = (11.16/2)/100 #in meters
        self.L = (2*2.13+13.84+13.35)/100 #in meters
        self.oldTheta = 0
        self.oldX = 0
        self.oldY = 0
        self.newTheta = 0
        self.newX = 0
        self.newY = 0
        
    def get_velRight(self, msg):

        self.velRight = (msg.data/60)*2*pi*self.R
        self.nowR = rospy.get_rostime().secs

    def get_velLeft(self, msg):

        self.velLeft = (msg.data/60)*2*pi*self.R
        self.nowL = rospy.get_rostime().secs
    
    def get_distR(self):

        dt = (self.nowR-self.beforeR)
        self.distanceR = self.velRight*dt
        return self.distanceR

    def get_distL(self):

        dt = (self.nowL-self.beforeL)
        self.distanceL = self.velLeft*dt
        return self.distanceL
    
    def get_quaternion_from_euler(roll, pitch, yaw):
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
        vl = self.R*(self.velRight+self.velLeft)/2
        w = self.R*(self.velRight-self.velLeft)/(2*self.L)
        
        #Pose Calc.
        distance_center = (self.distanceL + self.distanceR)/2
        dtheta = (self.distanceL - self.distanceR)/self.L

        self.newTheta = self.oldTheta+dtheta
        self.newX = self.oldX+distance_center*cos(self.oldTheta)
        self.newY = self.oldY+distance_center*cos(self.oldTheta)
        
        [qx, qy, qz, qw] = self.get_quaternion_from_euler(0,0,self.newTheta)
        now =rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.newX
        self.odom_msg.pose.pose.position.y = self.newY
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
            while not rospy.is_shutdown():
               odom.set_odom() 

        except rospy.ROSInterruptException: pass