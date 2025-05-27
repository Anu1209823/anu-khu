#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)
        self.cmd_vel_pub = rospy.Publisher('/anukhu/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/anukhu/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        rospy.spin()

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_vel_pub.publish(cmd)

    def move_robot(self, detections):
        TARGET_DISTANCE = 0.25
        MAX_LIN_VEL = 0.4
        MAX_ANG_VEL = 5.0
        LIN_KP = 1.2
        ANG_KP = 8.0

        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()

        if not detections:
            cmd.v = 0.0
            cmd.omega = 0.0
            rospy.loginfo("No AprilTag detected.")
        else:
            tag = detections[0]
            x = tag.transform.translation.x
            z = tag.transform.translation.z
            rospy.loginfo("Tag position: x = %.3f m, z = %.3f m", x, z)
            lin_error = z - TARGET_DISTANCE
            ang_error = -x
            cmd.v = max(min(LIN_KP * lin_error, MAX_LIN_VEL), -MAX_LIN_VEL)
            cmd.omega = max(min(ANG_KP * ang_error, MAX_ANG_VEL), -MAX_ANG_VEL)

        self.cmd_vel_pub.publish(cmd)

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

if __name__ == '__main__':
    try:
        Target_Follower()
    except rospy.ROSInterruptException:
        pass

