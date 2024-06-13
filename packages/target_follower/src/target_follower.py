#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class TargetFollower:
    """
    A class to control a robot to follow an April tag target.
    """
    def __init__(self):
        """
        Initialize the TargetFollower class.
        """
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        # Initialize publisher and subscriber with your robot's name
        self.cmd_vel_pub = rospy.Publisher('/shravel/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.detection_sub = rospy.Subscriber('/apriltag_detector_node/detections', AprilTagDetectionArray, self.detect_tag, queue_size=1)
        rospy.spin() # Spin forever but listen to message callbacks

    def detect_tag(self, msg):
        """
        Callback function for April Tag Detection.
        Moves the robot based on the detected April tag position.
        """
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        """
        Function called when the node is shutting down.
        Stops the robot before the node is terminated.
        """
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        """
        Sends zero velocity to stop the robot.
        """
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        """
        Moves the robot based on the detected April tag position.
        """
        if len(detections) == 0:
            self.stop_robot()
            return
        
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        
        x = detections.transform.translation.x
        y = detections.transform.translation.y
        z = detections.transform.translation.z
        rospy.loginfo("x, y, z: %f, %f, %f", x, y, z)
        rospy.sleep(1)
        
        if z > 0.15:
            cmd_msg.v = 0.2
            cmd_msg.omega = 0
        elif z < 0.10:
            cmd_msg.v = -0.2
            cmd_msg.omega = 0
        elif x > 0.05:
            cmd_msg.v = 0
            cmd_msg.omega = -0.4
        elif x < -0.05:
            cmd_msg.v = 0
            cmd_msg.omega = 0.4
        else:
            self.stop_robot()
            return
        
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(0.2)
        self.stop_robot()

def main():
    try:
        target_follower = TargetFollower()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

