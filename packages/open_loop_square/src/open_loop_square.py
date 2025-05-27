#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class DriveSquare:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        rospy.init_node('drive_square_node', anonymous=True)
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
            self.drive_square()
 
    def drive_square(self):
        for i in range(4):  # 4 sides
            # Drive straight
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.25  # speed in m/s
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Side {i+1}: Driving forward")
            rospy.sleep(6)  # 0.25 m/s * 6 s = 1.5 m

            # Turn 90°
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 3.14  # radians/sec
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Side {i+1}: Turning 90 degrees")
            rospy.sleep(0.55)  # ~90 degrees

        self.stop_robot()
        rospy.loginfo("Finished square. Robot stopped.")

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bot = DriveSquare()
        bot.run()
    except rospy.ROSInterruptException:
        pass

