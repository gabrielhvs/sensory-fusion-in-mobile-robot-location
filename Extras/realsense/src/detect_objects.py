import cv2
import imutils
import numpy as np
#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class Control_Ararajuba:
    
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('RobotTrainingNode', anonymous=True)
        self.initSubscribers()
        self.rate = rospy.Rate(10)
        

    def initSubscribers(self):
        
        #--- camera---
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback,  queue_size = 1)
        
        self.rate = rospy.Rate(10)
    
    def camera_callback(self,data):
        
        
        self.bridge = CvBridge()
       
        yellowLower = np.array([20, 80, 80], np.uint8)
        yellowUpper = np.array([32, 255, 255], np.uint8)
        try:
            #Conversão do sensor_msgs/Image to OpenCV Image
            cv2_image = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
            
            cv2_image = imutils.resize(cv2_image, width=600)
            hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
            
            maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
            cv2.imshow("Camera2", maskYellow)
            maskYellow = cv2.erode(maskYellow, None, iterations=2)
            maskYellow = cv2.dilate(maskYellow, None, iterations=2)
            
            #contornos
            cntYellow = cv2.findContours(maskYellow.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            
            centerYellow = 0
            if len(cntYellow) > 0:
                print (len(cntYellow))
                cYellow = min(cntYellow, key=cv2.contourArea)
                rectYellow = cv2.minAreaRect(cYellow)
                boxYellow = cv2.boxPoints(rectYellow)
                boxYellow = np.int0(boxYellow)
                MYellow = cv2.moments(cYellow)
                if MYellow['m00'] > 0:
                    M = MYellow
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centerYellow = (cx, cy)
                cv2.drawContours(cv2_image, [boxYellow], 0, (0, 255, 255), 2)
                pub = rospy.Publisher('detected', Point, queue_size=1)
                
                msg = Point()
                msg.x = cx * 0.11666
                msg.y = cy * 0.09777
                print (centerYellow)
                rospy.loginfo(msg)
                
                pub.publish(msg)
                
            else:
                pub2 = rospy.Publisher('detected', Point, queue_size=1)
                msg = Point()
                msg.x = 0
                msg.y = 0
                rospy.loginfo(msg)
                pub2.publish(msg)
            
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Camera", cv2_image)
        cv2.waitKey(3)

    
if __name__ == '__main__':
    try:
        robot = Control_Ararajuba()
        rospy.spin()
            
    except rospy.ROSInterruptException: pass


