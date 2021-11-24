#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import numpy as np


class G29Control():

    def __init__(self):
        # Mapping
        # 0 -> 450 degree == 0 -> 1
        ratio_constant = 7.854 # orientation:steering = 1:1  
        self.steering_ratio = ratio_constant / 2.0 # orientation:steering = 1:2

        # ROS
        # publish to the mobile_base controller
        self.g29_pub = rospy.Publisher('/base_controller/cmd_vel', Twist, queue_size=1)
        # subscribe to joystick inputs on topic "joy"
        self.joy_subscriber = rospy.Subscriber("G29/joy", Joy, self.g29_callback)
        

    def g29_callback(self, data):
        steering_input = data.axes[0]
        linear_vel_fwd = data.axes[2]
        linear_vel_bwd = data.axes[3]

        # Publsish
        twist = Twist()
        twist.linear.x = (linear_vel_fwd - linear_vel_bwd) / 2.0 # [-1, 1]
        twist.angular.z = self.steering_ratio * steering_input
        if twist.linear.x < 0:
            twist.angular.z = -twist.angular.z # flip direction when going backwards
        twist.angular.z = np.clip(twist.angular.z, -1.0, 1.0) # limit to [-1, 1]
        
        self.g29_pub.publish(twist)


if __name__ == "__main__":
    # launch falcon control
    rospy.init_node('g29_control_node')
    g29_control = G29Control()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
