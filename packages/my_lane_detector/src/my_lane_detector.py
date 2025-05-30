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
        rospy.loginfo("image_callback")

        # Convert ROS compressed image to OpenCV BGR image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Crop image to only show the road
        height, width, _ = img.shape
        cropped_img = img[int(height/2):, :]  # crop bottom half

        # Convert to HSV
        hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Filter white color
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 30, 255])
        white_mask = cv2.inRange(hsv_img, lower_white, upper_white)
        white_result = cv2.bitwise_and(cropped_img, cropped_img, mask=white_mask)

        # Filter yellow color
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        yellow_result = cv2.bitwise_and(cropped_img, cropped_img, mask=yellow_mask)

        # Canny edge detection
        edges = cv2.Canny(cropped_img, 50, 150)

        # Convert white/yellow masks to grayscale for Hough transform
        white_gray = cv2.cvtColor(white_result, cv2.COLOR_BGR2GRAY)
        yellow_gray = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)

        # Hough Transform
        lines_white = cv2.HoughLinesP(white_gray, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        lines_yellow = cv2.HoughLinesP(yellow_gray, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)

        # Draw lines on cropped image
        output_img = np.copy(cropped_img)
        if lines_white is not None:
            for line in lines_white:
                x1, y1, x2, y2 = line[0]
                cv2.line(output_img, (x1, y1), (x2, y2), (255, 255, 255), 2)
        if lines_yellow is not None:
            for line in lines_yellow:
                x1, y1, x2, y2 = line[0]
                cv2.line(output_img, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # Show results
        cv2.imshow("White Filtered", white_result)
        cv2.imshow("Yellow Filtered", yellow_result)
        cv2.imshow("Hough Lines", output_img)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = Lane_Detector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
