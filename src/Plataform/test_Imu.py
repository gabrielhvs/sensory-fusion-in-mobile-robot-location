from cmath import pi
import scipy
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
import pdb 




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
        self.pose = self.rospy.Subscriber('imu/data', Imu, self.get_Imu_filter, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def initvariables(self):
        self.R = 0.095
        self.L = 0.331
        
    def get_pose(self, msg):

        #Posição
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #Orientação em Quartenos
        Qx = msg.pose.pose.orientation.x
        Qy = msg.pose.pose.orientation.y
        Qz = msg.pose.pose.orientation.z
        Qw = msg.pose.pose.orientation.w
        #Orientação
        self.theta = atan2(2*(Qw*Qz+Qx*Qy),1-2*(Qy*Qy+Qz*Qz))
    
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

    def get_Imu_filter(self, msg):

        #Orientação em Quartenos
        Qxf = msg.orientation.x
        Qyf = msg.orientation.y
        Qzf = msg.orientation.z
        Qwf = msg.orientation.w
        self.thetaf = atan2(2*(Qwf*Qzf+Qxf*Qyf),1-2*(Qyf*Qyf+Qzf*Qzf))

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

            Orientation=[]
            thetaFO = 0
            thetaIO = 0
            i=0
            
            
            while i<1000:
                time.sleep(0.1)
                Orientation.append([180*robot.thetaf/pi,180*robot.thetai/pi])
                i=i+1
                print(str(180*robot.thetaf/pi)+'|'+str(180*robot.thetai/pi))
                #print(180*robot.thetai/pi)
            Orientation = np.array(Orientation)
            Orientation[:,0]= Orientation[:,0] #- (sum(Orientation[:,0]) / float(len(Orientation[:,0])))
            Orientation[:,1]= Orientation[:,1] #- (sum(Orientation[:,1]) / float(len(Orientation[:,1])))
            #plt.plot(PoseO[:,0],PoseO[:,1], label = "Odometria")
            plt.plot(Orientation[:,0],label = "Orientation from Filter")
            plt.plot(Orientation[:,1],label = "Orientation from IMU")
            plt.xlabel('Samples')
            plt.ylabel('Orientation')
            plt.title('Orientation Filter X Imu')
            plt.legend()
                      
            #for i in range(0,len(Orientation[:,0]),round(len(Orientation[:,0])/4)):
            #    robot.drawRobot(0,0,Orientation[:,0],0.02)
            #    robot.drawRobot(0,0,Orientation[:,1],0.02)

            plt.show()
        except rospy.ROSInterruptException: pass