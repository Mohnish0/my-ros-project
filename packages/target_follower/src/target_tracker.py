#!/usr/bin/env python3

import rospy
import time
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        # Initialize publisher and subscriber with your robot's name
        self.cmd_vel_pub = rospy.Publisher('/booty/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.tag_sub = rospy.Subscriber('/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        self.is_shutting_down = False
        self.last_detection_time = rospy.Time.now()
        self.target_lost_timeout = 1.0  # Timeout in seconds before considering the target lost

        rospy.spin()  # Spin forever but listen to message callbacks

    # April Tag Detection Callback
    def tag_callback(self, msg):
        if not self.is_shutting_down:
            self.move_robot(msg.detections)
            self.last_detection_time = rospy.Time.now()

    def clean_shutdown(self):
        self.is_shutting_down = True
        self.stop_robot()
        rospy.loginfo("System shutting down. Stopping robot...")

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        if len(detections) == 0 or (rospy.Time.now() - self.last_detection_time).to_sec() > self.target_lost_timeout:
            self.rotate_to_search()
            return

        target = detections[0]  # Assuming the closest tag is the target
        x = target.transform.translation.x
        y = target.transform.translation.y
        z = target.transform.translation.z

        rospy.loginfo("Target position: x=%.2f, y=%.2f, z=%.2f", x, y, z)

        if abs(x) > 0.05:
            self.rotate_to_align(x)
        else:
            self.move_forward()

    def rotate_to_search(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0
        cmd_msg.omega = 0.2
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(0.4)
        self.stop_robot()

    def rotate_to_align(self, x):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0
        if x > 0:
            cmd_msg.omega = -0.4
        else:
            cmd_msg.omega = 0.4
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(0.4)
        self.stop_robot()

    def move_forward(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.2
        cmd_msg.omega = 0
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass

