#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SubAndPub():

    def __init__(self):
        # ROS
        # subscribe to keyboard input
        self.subsciber = rospy.Subscriber("key", String, self.get_key_callback)
        # publish the control command
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)


    def get_key_callback(self, data):
        # Initialize published message
        twist = Twist()
        
        # Map to command
        key = data.data
        if key == "w":
            twist.linear.x = 1.0
        if key == "s":
            twist.linear.x = -1.0
        if key == "a":
            twist.angular.z = 1.0
        if key == "d":
            twist.angular.z = -1.0

        # rospy.loginfo("Send twist commad: Lx: ", twist.linear.x, ", Az: ", twist.angular.z)
        self.publisher.publish(twist)


if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('sub_and_pub_template_node')
    sub_and_pub = SubAndPub()

    # Set rate and run
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
