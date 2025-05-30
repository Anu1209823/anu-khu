#!/usr/bin/env python3
print("Python lane detector starting...")

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
    cropped = img[int(height/2):, :]

    # HSV conversion
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    cv2.imshow('HSV', hsv)  # (Optional, looks weird but you can see values)

    # White mask (tune these values as needed)
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 60, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow('White Mask', white_mask)

    # Apply mask to see the actual white lanes
    white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)
    cv2.imshow('White Lane Filter', white_result)

    cv2.waitKey(1)


if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass

