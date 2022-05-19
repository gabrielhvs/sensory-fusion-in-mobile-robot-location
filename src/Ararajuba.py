import scipy
from numpy import *
import numpy as np
import matplotlib.pyplot as plt
from math import pi
#!/usr/bin/env python
import rospy
import time
from math import pow, atan2, radians, sqrt, degrees, cos, sin, pi, tanh
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
        self.pose = self.rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.get_FilterPose, queue_size=10)
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
    
    def get_FilterPose(self, msg):

        #Posição
        self.xf = msg.pose.pose.position.x
        self.yf = msg.pose.pose.position.y
        self.zf = msg.pose.pose.position.z
        #Orientação em Quartenos
        self.Qxf = msg.pose.pose.orientation.x
        self.Qyf = msg.pose.pose.orientation.y
        self.Qzf = msg.pose.pose.orientation.z
        self.Qwf = msg.pose.pose.orientation.w
        #Covariance 
        # covariance  = msg.pose.pose.covariance   

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
        #Orientação
    
    def get_sonar_1(self, msg):
        #Range
        self.distance_1 = msg.range
    
    def get_sonar_2(self, msg):
        #Range
        self.distance_2 = msg.range
    
    def get_sonar_3(self, msg):
        #Range
        self.distance_3 = msg.range

    def printData(self):
        time.sleep(0.5)
        """
        print("Acelarações lineares:")
        print(" x:" + str(self.Acx))
        print(" y:" + str(self.Acy))
        print(" z:" + str(self.Acz) + "\n")
        
        print("Velocidades Angulares:")
        print(" x:" + str(self.Vax))
        print(" y:" + str(self.Vay))
        print(" z:" + str(self.Vaz)+ "\n")

        [self.roll, self.picth, self.yaw] = self.euler_from_quaternion(self.Qx, self.Qy, self.Qz, self.Qw)
        print("Orientações:")
        print(" roll:" + str(self.roll))
        print(" pitch:" + str(self.picth))
        print(" yaw:" + str(self.yaw))

        print("Posição Estimada (IMU-Odom):")
        print(" x:" + str(self.xf))
        print(" y:" + str(self.yf))
        print(" z:" + str(self.zf)+ "\n")

        [self.rollf, self.picthf, self.yawf] = self.euler_from_quaternion(self.Qxf, self.Qyf, self.Qzf, self.Qwf)
        print("Orientações Estimada (IMU-Odom):")
        print(" roll:" + str(self.rollf))
        print(" pitch:" + str(self.picthf))
        print(" yaw:" + str(self.yawf))
        """
        
        print("Sonar's distance: ")
        print(" Sonar1:" + str(self.distance_1))
        print(" Sonar2:" + str(self.distance_2))
        print(" Sonar3:" + str(self.distance_3))
        

        
if __name__ == '__main__':
        try:
            robot = Control_Ararajuba()
            while True:
                robot.printData()
            
        except rospy.ROSInterruptException: pass


