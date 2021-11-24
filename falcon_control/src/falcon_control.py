#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist, Vector3


class FalconControl():

    def __init__(self):
        # ROS
        # subscribe to falcon
        self.falcon_sub = rospy.Subscriber("falcon_pos", Point, self.falcon_callback, queue_size=1)
        # publish the command
        self.command_pub = rospy.Publisher("base_controller/cmd_vel", Twist, queue_size=1)
        
        self.feedback_pub = rospy.Publisher("set_falcon_force", Vector3, queue_size=1)

    def falcon_callback(self, data):
        # If falcon is at home position
        falcon_x, falcon_y, falcon_z = self.check_home_pos(data.x, data.y, data.z)
        # Map to vx, wz
        if (falcon_x==0.0) and (falcon_y==0.0) and (falcon_z==0.125):
            vx, wz = 0.0, 0.0
        else:
            fx, fy, fz = self.map_to_world(falcon_x, falcon_y, falcon_z, 2.0, 2.0, 2.0)
            vx = -fz
            wz = -fx
            if vx < 0:
                wz = -wz # flip direction when going backwards

        # Publsish
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = wz
        self.command_pub.publish(cmd_vel)
        
        feedback_mode = Vector3()
        feedback_mode.x = 1.5
        feedback_mode.y = 1.5
        feedback_mode.z = 1.5
        self.feedback_pub.publish(feedback_mode)

    def check_home_pos(self, x=0, y=0, z=0.125):
        # Check if falcon is around home position
        # if so, set to home position
        if abs(x) < 0.006:
            x = 0.0
        if abs(y) < 0.006:
            y = 0.0
        if abs(z-0.125) < 0.005:
            z = 0.125
        return x, y, z

    def map_to_world(self, falcon_x, falcon_y, falcon_z, range_x, range_y, range_z):
        # Map from falcon coordinate to world coordinate
        k_x = range_x/2 / 0.06 # [-0.06, 0.06] -> [-range/2, range/2]
        k_y = range_y/2 / 0.06 # [-0.06, 0.06] -> [-range/2, range/2]
        k_z = range_z/2 / 0.05 # [-0.05, 0.05] -> [-range/2, range/2]

        x = k_x * falcon_x          # [-0.06, 0.06]  -> [-range/2, range/2]
        y = k_y * falcon_y          # [-0.06, 0.06]  -> [-range/2, range/2]
        z = k_z *(falcon_z-0.125)   # [0.075, 0.175] -> [-range/2, range/2]
        
        return x, y, z


if __name__ == "__main__":
    # launch falcon control
    rospy.init_node('falcon_control_node')
    falcon_control = FalconControl()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
