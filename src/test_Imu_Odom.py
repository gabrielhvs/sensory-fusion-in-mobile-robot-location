from cmath import pi
from numpy import *
import numpy as np
import matplotlib.pyplot as plt
#!/usr/bin/env python
import rospy
import time
from math import atan2
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseWithCovarianceStamped 
import csv

class Control_Ararajuba:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('Jujuba', anonymous=True)
        self.initInstances()
        self.initSubscribers()
        self.initPublishers()
        self.initvariables()
        self.rate = rospy.Rate(10)
    
    def initInstances(self):
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        self.pose = self.rospy.Subscriber('imu/data_raw', Imu, self.get_Imu, queue_size=10)
        self.pose = self.rospy.Subscriber('/odom', Odometry, self.get_odom, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def initvariables(self):
        self.R = 0.095
        self.L = 0.331
    
    def get_odom(self, msg):

        #Posição
        self.xo = msg.pose.pose.position.x
        self.yo = msg.pose.pose.position.y
        #Orientação em Quartenos
        Qxo = msg.pose.pose.orientation.x
        Qyo = msg.pose.pose.orientation.y
        Qzo = msg.pose.pose.orientation.z
        Qwo = msg.pose.pose.orientation.w
        #Orientação
        self.thetao = atan2(2*(Qwo*Qzo+Qxo*Qyo),1-2*(Qyo*Qyo+Qzo*Qzo))
        
    def get_Imu(self, msg):

        #Orientação em Quartenos
        self.Qxi = msg.orientation.x
        self.Qyi = msg.orientation.y
        self.Qzi = msg.orientation.z
        self.Qwi = msg.orientation.w
        self.thetai = atan2(2*(self.Qwi*self.Qzi+self.Qxi*self.Qyi),1-2*(self.Qyi*self.Qyi+self.Qzi*self.Qzi))

    def drawRobot(self,x,y,q,s):
        p=[ [1,              1/7],  
        [-3/7,            1],  
        [-5/7,            6/7],       
        [-5/7,            5/7],     
        [-3/7,            2/7],     
        [-3/7,            0],       
        [-3/7,           -2/7],     
        [-5/7,           -5/7],     
        [-5/7,           -6/7],     
        [-3/7,           -1],       
            [1,             -1/7],     
            [1,              1/7 ]]
    
        
        p=[[s*i[0], s*i[1]] for i in p]
        p=[[i[0], i[1], 1] for i in p]
        r=[[math.cos(q),math.sin(q)],[math.sin(-q),math.cos(q)],[x,y]]
        c=np.matmul(p,r)
            
            
        X=c[:,0] 
        Y=c[:,1] 
        plt.plot(X,Y,'r-')  
    
        
if __name__ == '__main__':
        
        try:
            
            robot = Control_Ararajuba()
            time.sleep(0.5)
            #pdb.set_trace()

            i=0
            v = 3*(2*pi)/60
            w = 2*(2*pi)/60
            a = input("Press Enter")

            file = open('test.csv', 'w')
            writer = csv.writer(file)

            while i<1000:
                robot.vel_msg.linear.x = v
                robot.vel_msg.angular.z = w
                robot.velocity_publisher.publish(robot.vel_msg)
                time.sleep(0.1)
                writer.writerow([robot.rospy.get_rostime().secs,robot.rospy.get_rostime().nsecs,robot.xo, robot.yo, robot.thetao, robot.thetai])
                i=i+1
                time.sleep(0.1)
                print(i)
                
            print("here!")
            robot.vel_msg.linear.x = 0
            robot.vel_msg.angular.z = 0
            robot.velocity_publisher.publish(robot.vel_msg)
            file.close()

        except rospy.ROSInterruptException: pass