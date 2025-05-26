#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class Drive_Square:
    def __init__(self):
        # Initialize Twist message
        self.cmd_msg = Twist2DStamped()

        # Start ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Update this to your actual Duckiebot name
        self.vehicle_name = "anukhupi"

        # Publisher to send motion commands
        self.pub = rospy.Publisher(
            f"/{self.vehicle_name}/car_cmd_switch_node/cmd", 
            Twist2DStamped, 
            queue_size=1
        )

        # Subscriber to listen for FSM mode
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
            self.move_robot()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_robot(self):
        for i in range(4):  # 4 sides of a square
            # Drive forward
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.3
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Side {i+1}: Driving forward")
            rospy.sleep(1.5)

            # Turn 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = 3.14  # ~180 deg/sec
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Side {i+1}: Turning")
            rospy.sleep(0.5)  # ~90 degrees turn

        self.stop_robot()
        rospy.loginfo("Square completed. Robot stopped.")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

