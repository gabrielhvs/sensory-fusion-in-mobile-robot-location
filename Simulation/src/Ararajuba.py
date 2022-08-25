import scipy
from numpy import *
import numpy as np
import matplotlib.pyplot as plt
#!/usr/bin/env python
import rospy
import time
from math import atan2
from tabulate import tabulate
#from geometry_msgs.msg  import Twist
from geometry_msgs.msg import Twist
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
        self.initvariables()
        self.rate = rospy.Rate(10)
    
    def initInstances(self):
        self.vel_msg = Twist()
        self.rate = rospy.Rate(10)
    
    def initSubscribers(self):
        self.pose = self.rospy.Subscriber('/odom', Odometry, self.get_odom, queue_size=10)
        self.pose = self.rospy.Subscriber('/imu_1', Imu, self.get_imu, queue_size=10)
        self.pose = self.rospy.Subscriber('/sonar/sonar_1', Range, self.get_sonar_1, queue_size=10)
        self.pose = self.rospy.Subscriber('/sonar/sonar_2', Range, self.get_sonar_2, queue_size=10)
        self.pose = self.rospy.Subscriber('/sonar/sonar_3', Range, self.get_sonar_3, queue_size=10)
        self.pose = self.rospy.Subscriber('/base_pose_ground_truth', Odometry, self.get_pose, queue_size=10)
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def initvariables(self):
        self.R = 0.095
        self.L = 0.331

    def euler_from_quaternion(self, x, y, z, w):
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 

    def get_pose(self, msg):

        #Posição
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        #Orientação em Quartenos
        Qx = msg.pose.pose.orientation.x
        Qy = msg.pose.pose.orientation.y
        Qz = msg.pose.pose.orientation.z
        Qw = msg.pose.pose.orientation.w
        #Orientação
        [self.rollg, self.picthg, self.yawg] = self.euler_from_quaternion(Qx, Qy, Qz, Qw)
    
    def get_odom(self, msg):

        #Posição
        self.xo = msg.pose.pose.position.x
        self.yo = msg.pose.pose.position.y
        self.zo = msg.pose.pose.position.z
        #Orientação em Quartenos
        Qxo = msg.pose.pose.orientation.x
        Qyo = msg.pose.pose.orientation.y
        Qzo = msg.pose.pose.orientation.z
        Qwo = msg.pose.pose.orientation.w
        #Orientação
        [self.rollo, self.pictho, self.yawo] = self.euler_from_quaternion(Qxo, Qyo, Qzo, Qwo)
      
    def get_imu(self, msg):

        #Aceleration
        self.Acx = msg.linear_acceleration.x
        self.Acy = msg.linear_acceleration.y
        self.Acz = msg.linear_acceleration.z
        
        #Velocity angular
        self.Vax = msg.angular_velocity.x
        self.Vay = msg.angular_velocity.y
        self.Vaz = msg.angular_velocity.z

        #Orientação em Quartenos
        self.Qx = msg.orientation.x
        self.Qy = msg.orientation.y
        self.Qz = msg.orientation.z
        self.Qw = msg.orientation.w
        
    def get_sonar_1(self, msg):
        #Range
        self.distance_1 = msg.range
    
    def get_sonar_2(self, msg):
        #Range
        self.distance_2 = msg.range
    
    def get_sonar_3(self, msg):

        self.distance_3 = msg.range

    def printData(self):
        
        time.sleep(0.5)

        [self.roll, self.picth, self.yaw] = self.euler_from_quaternion(self.Qx, self.Qy, self.Qz, self.Qw)

        data = [[1, "roll: "+str(round(self.roll,4)), "x: "+str(round(self.xo,4)), "x: "+str(round(self.x,4)),  "dist-1: "+str(round(self.distance_1,2))],
                    [2, "pitch: "+str(round(self.picth,4)), "y: "+str(round(self.yo,4)), "y: "+str(round(self.y,4)),  "dist-2: "+str(round(self.distance_2,2))],
                    [3, "yaw: "+str(round(self.yaw,4)), "z: "+str(round(self.zo,4)), "z: "+str(round(self.z,4)),  "dist-3: "+str(round(self.distance_3,2))],
                    [4,"Aclx: "+str(round(self.Acx,4)), "roll: "+str(round(self.rollo,4)), "roll: "+str(round(self.rollg,4)), 0],
                    [5,"Acly: "+str(round(self.Acy,4)), "pitch: "+str(round(self.pictho,4)),  "pitch: "+str(round(self.picthg,4)), 0],
                    [6,"Aclz: "+str(round(self.Acz,4)), "yaw: "+str(round(self.yawo,4)), "yaw: "+str(round(self.yawg,4)), 0]]
        print(tabulate(data, headers=["Sensor:", "Imu", "Odom", "GroundTruth","Sonars"]))
        
        
if __name__ == '__main__':
        try:
            while True:
                robot = Control_Ararajuba()
                robot.printData()
        except rospy.ROSInterruptException: pass
