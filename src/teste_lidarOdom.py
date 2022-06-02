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




class Control_Ararajuba:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('RobotTrainingNode', anonymous=True)
        self.initInstances()
        self.initSubscribers()
        self.initPublishers()
        self.rate = rospy.Rate(10)
    
    def initInstances(self):
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        self.pose = self.rospy.Subscriber('/pose2D', Pose2D, self.get_poseScan, queue_size=10)
        self.pose = self.rospy.Subscriber('/base_pose_ground_truth', Odometry, self.get_pose, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def get_poseScan(self,msg):
        self.xscan = msg.x
        self.yscan = msg.y
        self.thetascan = msg.theta
        
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
            time.sleep(0.1)
            
            PoseF=[]
            PoseO=[]
            PoseGT=[]
            i=0
            while i<200:
                time.sleep(0.1)
                PoseGT.append([robot.x,robot.y,robot.theta])
                PoseF.append([robot.xscan,robot.yscan,robot.thetascan])
                i=i+1
                print("Iterações: "+ str(i))
            
            PoseGT = np.array(PoseGT)
            PoseF = np.array(PoseF)
            
            plt.plot(PoseGT[:,0],PoseGT[:,1],label = "Ground_Truth")
            plt.plot(PoseF[:,0],PoseF[:,1],label = "LiDAR")
            plt.xlabel('Posição eixo X')
            plt.ylabel('Posição eixo Y')
            plt.title('Posição Ground_Truth X Odometria LiDAR')
            plt.legend()
                      
            for i in range(0,len(PoseGT[:,0]),round(len(PoseGT[:,0])/20)):
                robot.drawRobot(PoseGT[i,0],PoseGT[i,1],PoseGT[i,2],0.02)
                robot.drawRobot(PoseF[i,0],PoseF[i,1],PoseF[i,2],0.02)

            plt.show()
        except rospy.ROSInterruptException: pass
