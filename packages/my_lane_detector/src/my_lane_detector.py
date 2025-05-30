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
    # Convert to OpenCV image
    img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    # Show the original image
    cv2.imshow('Original', img)
    cv2.waitKey(1)


    def run(self):
        rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass

