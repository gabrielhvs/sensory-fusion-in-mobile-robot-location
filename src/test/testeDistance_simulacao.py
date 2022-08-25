from math import pi
#!/usr/bin/env python
import rospy
import time
from math import atan2, pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Control_Ararajuba:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('Ararajuba')
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
        self.rate = rospy.Rate(10)
        
    def initPublishers(self):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def initvariables(self):
        self.distance_tolerance = 0.1
        self.distance_toleranceA = 0.2

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
    
        
    def Distance_Static(self, d):
        time.sleep(0.5)
        
        while ((d-self.xo)>0.1):
    
            self.vel_msg.linear.x = 2
            print(self.xo)
            self.velocity_publisher.publish(self.vel_msg)
        
        self.vel_msg.linear.x = 0
        print(self.xo)
        self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
        try:
            robot = Control_Ararajuba()
            robot.Distance_Static(input("Type the speed here: "))
            
            
        except rospy.ROSInterruptException: pass


