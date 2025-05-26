#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class DriveStraight:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        rospy.init_node('drive_straight_node', anonymous=True)
        self.vehicle_name = "anukhu"

        self.pub = rospy.Publisher(
            f"/{self.vehicle_name}/car_cmd_switch_node/cmd", 
            Twist2DStamped, 
            queue_size=1
        )

        rospy.Subscriber(
            f"/{self.vehicle_name}/fsm_node/mode", 
            FSMState, 
            self.fsm_callback, 
            queue_size=1
        )

    def fsm_callback(self, msg):
        rospy.loginfo(f"FSM mode received: {msg.state}")
        if msg.state == "NORMAL_LANE_FOLLOWING":
            rospy.sleep(1)
            self.drive_forward()

    def drive_forward(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.25     # Adjust if too fast or too slow
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Driving straight for 8 seconds...")
        rospy.sleep(8.0)

        self.stop_robot()
        rospy.loginfo("Robot stopped.")

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bot = DriveStraight()
        bot.run()
    except rospy.ROSInterruptException:
        pass

