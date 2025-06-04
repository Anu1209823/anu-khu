#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class DriveSquare:
    def __init__(self):
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
        self.in_motion = False  # Prevent re-triggering

    def fsm_callback(self, msg):
        rospy.loginfo(f"FSM mode received: {msg.state}")
        # Only run if entering LANE_FOLLOWING and not already running
        if msg.state == "LANE_FOLLOWING" and not self.in_motion:
            self.in_motion = True
            rospy.sleep(1)  # Give robot a moment to stabilize
            self.drive_square()
        elif msg.state != "LANE_FOLLOWING":
            self.in_motion = False  # Reset when leaving state

    def drive_square(self):
        cmd_msg = Twist2DStamped()
        for i in range(4):  # 4 sides of the square
            # Drive straight
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.20  # meters per second
            cmd_msg.omega = 0.0
            rospy.loginfo(f"Side {i+1}: Driving forward")
            self.pub.publish(cmd_msg)
            rospy.sleep(3)  # Move straight (adjust if needed)

            # Turn 90 degrees
            cmd_msg.header.stamp = rospy.Time.now()
            cmd_msg.v = 0.0
            cmd_msg.omega = 3.14  # rad/sec (adjust if turns are too wide/tight)
            rospy.loginfo(f"Side {i+1}: Turning 90 degrees")
            self.pub.publish(cmd_msg)
            rospy.sleep(0.20)  # Turn in place (adjust if needed)

        self.stop_robot()
        rospy.loginfo("Finished square. Robot stopped.")

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.pub.publish(cmd_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        bot = DriveSquare()
        bot.run()
    except rospy.ROSInterruptException:
        pass

