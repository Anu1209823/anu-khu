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
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 110])
        upper_white = np.array([180, 70, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5,5), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        cv2.imshow('White Mask', white_mask)

        edges = cv2.Canny(white_mask, 50, 150)
        cv2.imshow('Edges', edges)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        lane_img = img.copy()
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(lane_img, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.imshow('Lane Lines', lane_img)
        cv2.waitKey(1)


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = Lane_Detector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
