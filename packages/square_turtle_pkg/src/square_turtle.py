#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)

    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Turtles are great at drawing squares!")

    rate = rospy.Rate(20)  # 20 Hz

    while not rospy.is_shutdown():
        # Move forward for 2 seconds
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 2.0
        for i in range(40):  # 2 seconds at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Rotate 90 degrees
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.57  # 1.57 rad/s for 90 degrees
        for i in range(20):  # 1 second at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Move forward for 2 seconds
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 2.0
        for i in range(40):  # 2 seconds at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Rotate 90 degrees
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.57  # 1.57 rad/s for 90 degrees
        for i in range(20):  # 1 second at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Move forward for 2 seconds
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 2.0
        for i in range(40):  # 2 seconds at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Rotate 90 degrees
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.57  # 1.57 rad/s for 90 degrees
        for i in range(20):  # 1 second at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Move forward for 2 seconds
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 2.0
        for i in range(40):  # 2 seconds at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

        # Rotate 90 degrees
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.57  # 1.57 rad/s for 90 degrees
        for i in range(20):  # 1 second at 20 Hz
            velocity_publisher.publish(cmd_vel_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass
