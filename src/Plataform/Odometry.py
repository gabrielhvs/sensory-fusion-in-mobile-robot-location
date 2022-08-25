from cmath import pi
from numpy import *
import numpy as np
import csv
import matplotlib.pyplot as plt
#!/usr/bin/env python
import rospy
import time
from math import pi,cos, sin
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
        self.timeLeftA = (float(self.rospy.get_rostime().secs)%10000)+(float(self.rospy.get_rostime().nsecs)/(10**9))
        self.timeRightA = (float(self.rospy.get_rostime().secs)%10000)+(float(self.rospy.get_rostime().nsecs)/(10**9))

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
        self.L = (2*2.13+13.84+13.35)/200 #in meters
        self.Theta = 0
        self.X = 0
        self.Y = 0
        
    def get_velRight(self, msg):

        self.velRight = (msg.data/60)*2*pi*self.R
        self.nowRights = odom.rospy.get_rostime().secs     
        self.nowRightns = odom.rospy.get_rostime().nsecs

        
    def get_velLeft(self, msg):

        self.velLeft = (msg.data/60)*2*pi*self.R
        self.nowLefts = odom.rospy.get_rostime().secs    
        self.nowLeftns = odom.rospy.get_rostime().nsecs

   
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def set_odom(self):
        
        #Get time 
        timeRight=(self.nowRights%10000)+(self.nowRightns/(10**9))
        timeLeft=(self.nowLefts%10000)+(self.nowLeftns/(10**9))

        #Get Dist.
        distLeft = self.velLeft * (timeLeft - self.timeLeftA)
        self.timeLeftA = timeLeft
        distRight = self.velRight * (timeRight - self.timeRightA)
        self.timeRightA = timeRight

        vl = ((self.velRight+self.velLeft))/2
        w= ((self.velRight-self.velLeft))/(2*self.L)
        
        #Pose Calc.
        self.distance_center = (distLeft + distRight)/2
        self.dtheta = (distRight - distLeft)/(2*self.L)

        self.X = self.X+self.distance_center*cos(self.Theta)
        self.Y = self.Y+self.distance_center*sin(self.Theta)
        self.Theta = self.Theta+self.dtheta
        print(self.X)
        
        [qx, qy, qz, qw] = self.get_quaternion_from_euler(0,0,self.Theta)   
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
        return self.X,self.Y,self.Theta

                
if __name__ == '__main__':
        
        try:
            odom = Calc_Odom()
            time.sleep(0.1)
            count=0
            while not rospy.is_shutdown():
               X,Y,Theta = odom.set_odom() 
               time.sleep(0.1)

        except rospy.ROSInterruptException: pass
