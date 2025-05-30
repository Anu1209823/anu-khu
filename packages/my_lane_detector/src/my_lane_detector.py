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
        # Convert ROS message to OpenCV image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = img.shape

        # 1. Crop the bottom half
        cropped = img[int(height/2):, :]

        # 2. Convert to HSV
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # 3. Filter for White Pixels
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)
        cv2.imshow('White Lane Filter', white_result)

        # 4. Filter for Yellow Pixels
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)
        cv2.imshow('Yellow Lane Filter', yellow_result)

        # 5. Hough Transform for White
        white_edges = cv2.Canny(white_result, 50, 150)
        lines_white = cv2.HoughLinesP(white_edges, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=10)
        white_lines_img = np.copy(cropped)
        if lines_white is not None:
            for line in lines_white:
                x1, y1, x2, y2 = line[0]
                cv2.line(white_lines_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
        cv2.imshow('White Hough Lines', white_lines_img)

        # 6. Hough Transform for Yellow
        yellow_edges = cv2.Canny(yellow_result, 50, 150)
        lines_yellow = cv2.HoughLinesP(yellow_edges, 1, np.pi/180, threshold=30, minLineLength=30, maxLineGap=10)
        yellow_lines_img = np.copy(cropped)
        if lines_yellow is not None:
            for line in lines_yellow:
                x1, y1, x2, y2 = line[0]
                cv2.line(yellow_lines_img, (x1, y1), (x2, y2), (0, 255, 255), 3)
        cv2.imshow('Yellow Hough Lines', yellow_lines_img)

        # 7. Combine All Lines on One Image
        all_lines_img = np.copy(cropped)
        if lines_white is not None:
            for line in lines_white:
                x1, y1, x2, y2 = line[0]
                cv2.line(all_lines_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
        if lines_yellow is not None:
            for line in lines_yellow:
                x1, y1, x2, y2 = line[0]
                cv2.line(all_lines_img, (x1, y1), (x2, y2), (0, 255, 255), 3)
        cv2.imshow('All Lane Lines', all_lines_img)

        cv2.waitKey(1)

    def run(self):
        rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass

