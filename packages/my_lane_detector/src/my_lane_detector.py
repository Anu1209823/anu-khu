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
        cropped = img[int(height/2):, :]
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        # Detect lane lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        line_img = np.copy(cropped)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_img, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.imshow('Lane Lines', line_img)
        cv2.waitKey(1)




    def run(self):
        rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass

