#!/usr/bin/env python
import numpy as np
import imutils
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy      
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped



class Ros_Camera:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('camera')
        self.initPublishers()
        self.rate = rospy.Rate(10)

    def initPublishers(self):
        self.image_publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.point_publisher = rospy.Publisher('/detected', PointStamped, queue_size=1)
        self.rate = rospy.Rate(10)

    def detect(self,data):
        
        
        self.bridge = CvBridge()
       
        yellowLower = np.array([20, 80, 80], np.uint8)
        yellowUpper = np.array([32, 255, 255], np.uint8)

        try:
            
            #Conversão do sensor_msgs/Image to OpenCV Image
            cv2_image = data
            
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
                
                #pub = rospy.Publisher('detected', Point, queue_size=1)
                
                msg = PointStamped()
                msg.point.x = cx * 0.11666
                msg.point.y = cy * 0.09777
                msg.header.frame_id= 'camera_link'
                print (centerYellow)
                rospy.loginfo(msg)
                
                self.point_publisher.publish(msg)
                
            else:
            
                msg = PointStamped()
                msg.point.x = 0
                msg.point.y = 0
                msg.header.frame_id= 'camera_link'
                rospy.loginfo(msg)
                self.point_publisher.publish(msg)
            
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Camera", cv2_image)
        cv2.waitKey(3)


cam = cv2.VideoCapture(0)

if __name__ == '__main__':
        
    try:
        camera = Ros_Camera()

        while not rospy.is_shutdown():
            # Capture the video frame
            # by frame
            ret, frame = cam.read()
            msg_frame = CvBridge().cv2_to_imgmsg(frame, "bgr8")
            msg_frame.header.frame_id = 'camera_link'
            camera.detect(frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            camera.image_publisher.publish(msg_frame)
            time.sleep(0.1)

    except rospy.ROSInterruptException: pass
