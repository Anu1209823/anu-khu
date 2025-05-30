#!/usr/bin/env python3

import sys
import time
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        # Subscribe to Anukhu's camera image topic
        self.image_sub = rospy.Subscriber('/anukhu/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)

        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        cropped = img  # No cropping; use the full image

        # Convert to HSV color space
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # White mask
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        cv2.imshow('White Mask', white_mask)

        # Show only the white lane in color
        white_lane = cv2.bitwise_and(cropped, cropped, mask=white_mask)
        cv2.imshow('White Lane Detected', white_lane)

        cv2.waitKey(1)


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = Lane_Detector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
