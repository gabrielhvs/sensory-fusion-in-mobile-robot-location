#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import curses
import os

def main(win):
    rospy.init_node('p3dx_teleop', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = 0.3
    speed_angular = 0.4
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    win.nodelay(True)
    key=""
    win.clear()                
    win.addstr("Detected key:")
    while 1:          
        try:                 
           key = win.getkey()   
           win.clear()                
           win.addstr("Detected key:")
           win.addstr(str(key))
           if ("KEY_UP" == str(key)):
               vel_msg.linear.x = abs(speed)
           elif ("KEY_DOWN" == str(key)):
               vel_msg.linear.x = -abs(speed)
           
           if ("KEY_RIGHT" == str(key)):
               vel_msg.angular.z = -abs(speed_angular)
           elif ("KEY_LEFT" == str(key)):
               vel_msg.angular.z = abs(speed_angular)
           
           if ("KEY_BACKSPACE" == str(key)):
               vel_msg.linear.x = 0
               vel_msg.angular.z = 0


           velocity_publisher.publish(vel_msg)      
           
           if key == os.linesep:
              break           
        except Exception as e:
           # No input   
           pass         

curses.wrapper(main)