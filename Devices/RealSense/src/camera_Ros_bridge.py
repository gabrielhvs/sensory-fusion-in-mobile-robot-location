#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Ros_Camera:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('camera')
        self.initPublishers()
        
        self.rate = rospy.Rate(10)

    def initPublishers(self):
        self.image_publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.rate = rospy.Rate(10)


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
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            #camera.image_msg.data=
            camera.image_publisher.publish(msg_frame)
            time.sleep(0.1)

    except rospy.ROSInterruptException: pass
