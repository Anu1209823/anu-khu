#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def draw_square(velocity_publisher):
    cmd_vel_msg = Twist()

    for i in range(4):
        # Move forward
        cmd_vel_msg.linear.x = 2.0
        cmd_vel_msg.angular.z = 0.0
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(2)

        # Stop
        cmd_vel_msg.linear.x = 0.0
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(0.5)

        # Rotate 90 degrees
        cmd_vel_msg.angular.z = 1.57  # ~90 degrees/sec
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(1)

        # Stop
        cmd_vel_msg.angular.z = 0.0
        velocity_publisher.publish(cmd_vel_msg)
        rospy.sleep(0.5)

def move_turtle_square_loop():
    rospy.init_node('turtle_square_loop_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Starting square loop... Press Ctrl+C to stop.")

    rate = rospy.Rate(0.1)  # Run a full square roughly every 10 seconds

    while not rospy.is_shutdown():
        draw_square(velocity_publisher)
        rate.sleep()  # slight pause before repeating

if _name_ == '_main_':
    try:
        move_turtle_square_loop()
    except rospy.ROSInterruptException:
        pass