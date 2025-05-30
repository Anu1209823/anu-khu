#!/usr/bin/env python3

# Python Libs
import sys, time

# numpy
import numpy as np

# OpenCV
import cv2
from cv_bridge import CvBridge

# ROS Libraries
import rospy
import roslib

# ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()
        # Subscribe to the camera topic for your robot 'anukhu'
        self.image_sub = rospy.Subscriber(
            '/anukhu/camera_node/image/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1
        )
        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = img.shape
        cropped = img[int(height*0.4):, :]   # Try with more of the image for debugging
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # Tune your white threshold here
        lower_white = np.array([0, 0, 120])
        upper_white = np.array([180, 70, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        cv2.imshow('White Mask', white_mask)

        # Tune your yellow threshold here
        lower_yellow = np.array([10, 40, 40])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.imshow('Yellow Mask', yellow_mask)

        cv2.waitKey(1)



    def run(self):
        rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass

