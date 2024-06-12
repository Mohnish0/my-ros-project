#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)

        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/booty/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/booty/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

    # Robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # Wait for a sec for the node to be ready
            self.move_robot()

    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin()  # keeps node from exiting until node has shutdown

    # Robot drives in a square and then stops
    def move_robot(self):
        # Drive forward for 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = -0.3  # straight line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(1)  

        # Rotate 90 degrees clockwise
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = -0.3  # rotate clockwise
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Rotate clockwise!")
        rospy.sleep(1)  

        # Drive forward for 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = -0.3
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(1)  # straight line driving time

        # Rotate 90 degrees clockwise
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = -0.3
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Rotate clockwise!")
        rospy.sleep(1)  # rotation time

        # Drive forward for 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = -0.3
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(1)  # straight line driving time

        # Rotate 90 degrees clockwise
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = -0.3
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Rotate clockwise!")
        rospy.sleep(1)  # rotation time

        # Drive forward for 1 meter
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = -0.3
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(1)  # straight line driving time

        # Rotate 90 degrees clockwise
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = -0.3
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Rotate clockwise!")
        rospy.sleep(1)  # rotation time

        self.stop_robot()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
