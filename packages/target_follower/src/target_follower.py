#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Seeker:
    def __init__(self):
        rospy.init_node('target_seeker_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.vehicle_name = "anukhu"
        self.cmd_pub = rospy.Publisher(
            f"/{self.vehicle_name}/car_cmd_switch_node/cmd", 
            Twist2DStamped, 
            queue_size=1
        )
        rospy.Subscriber(
            f"/{self.vehicle_name}/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        self.seeking = True
        self.rate = rospy.Rate(10)  # 10 Hz

        self.run_loop()

    def run_loop(self):
        while not rospy.is_shutdown():
            if self.seeking:
                self.rotate_in_place()
            self.rate.sleep()

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            rospy.loginfo("No tag detected – seeking...")
            self.seeking = True
        else:
            rospy.loginfo("Tag detected – stopping rotation.")
            self.seeking = False
            self.stop_robot()

    def rotate_in_place(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 1.5  # Adjust this speed if needed
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)

    def clean_shutdown(self):
        rospy.loginfo("Shutdown initiated, stopping robot...")
        self.stop_robot()

if __name__ == '__main__':
    try:
        Target_Seeker()
    except rospy.ROSInterruptException:
        pass

