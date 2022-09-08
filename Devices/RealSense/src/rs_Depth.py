from email import message_from_binary_file
import queue
from socket import MsgFlag
from unicodedata import name
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image , CameraInfo
import time

from cv_bridge import CvBridge, CvBridgeError


class Ros_Camera:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('camera')
        self.initPublishers()
        
        self.rate = rospy.Rate(10)

    def initPublishers(self):
        self.image_publisher = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_publisher = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
        self.color_camera_info = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size = 10)
        self.rate = rospy.Rate(10)


if __name__ == '__main__':

    
    camera= Ros_Camera()

    width = 640
    height = 480

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    print("Depth Scale is: ", depth_scale)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            
            if not depth_frame or not color_frame:
                continue

            # convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            msg_frame_depth = CvBridge().cv2_to_imgmsg(depth_image, "16UC1")
            msg_frame_depth.header.frame_id = 'camera_depth_link'
            
            msg_frame = CvBridge().cv2_to_imgmsg(color_image, "bgr8")
            msg_frame.header.frame_id = 'camera_link'
            
            camera_info = CameraInfo()
            
            camera_info.height=480
            camera_info.width= 640
            camera_info.distortion_model= "plumb_bob"
            camera_info.D= [0.0, 0.0, 0.0, 0.0, 0.0]
            camera_info.K= [617.8533325195312, 0.0, 314.3265380859375, 0.0, 617.5131225585938, 241.8899688720703, 0.0, 0.0, 1.0]
            camera_info.R= [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info.P= [617.8533325195312, 0.0, 314.3265380859375, 0.0, 0.0, 617.5131225585938, 241.8899688720703, 0.0, 0.0, 0.0, 1.0, 0.0]
        

            camera.image_publisher.publish(msg_frame)
            camera.depth_publisher.publish(msg_frame_depth)
            camera.color_camera_info.publish(camera_info)

            depth = depth_image[320,240].astype(float)*depth_scale

            cv2.imshow('rgb', color_image)
            cv2.imshow('depth', depth_colormap)
            #print(f'Depth: {depth} m')

            if cv2.waitKey(1) == ord("q"):
                break

    finally:
        pipeline.stop()