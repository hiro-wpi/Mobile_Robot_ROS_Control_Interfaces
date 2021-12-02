#!/usr/bin/env python

# Task space of falcon limits ::
# Z limits : 0.074 (Inside) to 0.176 (Outside)
# Y limits : -0.056 (Down) to 0.062 (Up)
# X limits : -0.056 (Left) to 0.062 (Right)

import rospy
from geometry_msgs.msg import Point, Twist, Vector3
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
COLLISION_CLEARANCE = 1
    

class FalconControl():
    def __init__(self):
        # ROS
        # subscribe to falcon
        self.falcon_sub = rospy.Subscriber("falcon_pos", Point, self.falcon_callback, queue_size=1)
        self.laser_subscriber = rospy.Subscriber("base_scan", LaserScan, self.get_feedback)
        
        # publish the command
        self.command_pub = rospy.Publisher("base_controller/cmd_vel", Twist, queue_size=1)
        self.centering_pub = rospy.Publisher("set_falcon_haptic_mode", Int8, queue_size=1)
        self.feedback_pub = rospy.Publisher("set_falcon_force", Vector3, queue_size=1)
        self.feedback_flag = 0
        self.laser_range = []

    def get_feedback(self, data):
        self.laser_range = data.ranges
        temp = self.laser_range[160:180]
        # print(temp)
        # Center laser data start index : 82 and end index :110
        for i in temp:
            if i > 0.2 and i <= COLLISION_CLEARANCE-0.5:
                self.feedback_flag = 1
            elif i>COLLISION_CLEARANCE-0.5 and i <= COLLISION_CLEARANCE:
                self.feedback_flag = 2
            elif i <= COLLISION_CLEARANCE+0.5 and i> COLLISION_CLEARANCE:
                self.feedback_flag = 3
            elif i <= COLLISION_CLEARANCE+1 and i > COLLISION_CLEARANCE+0.5:
                self.feedback_flag = 4
            else:
                self.feedback_flag = 0   

    def falcon_callback(self, data):
        # If falcon is at home position
        falcon_x, falcon_y, falcon_z = self.check_home_pos(data.x, data.y, data.z)
        # Map to vx, wz
        if (falcon_x==0.0) and (falcon_y==0.0) and (falcon_z==0.125):
            vx, wz = 0.0, 0.0
        else:
            fx, fy, fz = self.map_to_world(falcon_x, falcon_y, falcon_z, 2.0, 2.0, 2.0)
            vx = -fz
            wz = -fx*2
            if vx < 0:
                wz = -wz # flip direction when going backwards
            
        # Publsish
        centering_mode = Int8()
        centering_mode.data = 0
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = wz
        self.command_pub.publish(cmd_vel)
        
        print("Feedback flag : ",self.feedback_flag)
        feedback_mode = Vector3()
        if self.feedback_flag == 4:
            feedback_mode.x = 0.0
            feedback_mode.y = 0.0
            feedback_mode.z = 1.5
            centering_mode.data = 2
            self.feedback_pub.publish(feedback_mode)
        elif self.feedback_flag == 3:
            feedback_mode.x = 0.0
            feedback_mode.y = 0.0
            feedback_mode.z = 2.5
            centering_mode.data = 2
            self.feedback_pub.publish(feedback_mode)
        elif self.feedback_flag == 2:
            feedback_mode.x = 0.0
            feedback_mode.y = 0.0
            feedback_mode.z = 3.5
            centering_mode.data = 2
            self.feedback_pub.publish(feedback_mode)
        elif self.feedback_flag == 1:
            feedback_mode.x = 0.0
            feedback_mode.y = 0.0
            feedback_mode.z = 5
            centering_mode.data = 2
            self.feedback_pub.publish(feedback_mode)
        elif self.feedback_flag == 0:
            feedback_mode.x = 0.0
            feedback_mode.y = 0.0
            feedback_mode.z = 0.0
            centering_mode.data = 0
            self.centering_pub.publish(centering_mode)
        
        if centering_mode == 0:
            self.centering_pub.publish(centering_mode)
        elif centering_mode == 2:
            self.centering_pub.publish(centering_mode)
            self.feedback_pub.publish(feedback_mode)
        
        #falcon_x, falcon_y, falcon_z = self.check_home_pos(data.x, data.y, data.z) 
        # feedback_mode.x = 0.0
        # feedback_mode.y = 0.0
        # feedback_mode.z = 0.0
        # self.feedback_pub.publish(feedback_mode)
        # print("x : {}, y : {}, z : {}".format(feedback_mode.x, feedback_mode.y, feedback_mode.z))
        # time.sleep(3)

        # feedback_mode.x = 0.0
        # feedback_mode.y = -1.0
        # feedback_mode.z = 0.0
        # self.feedback_pub.publish(feedback_mode)
        # print("x : {}, y : {}, z : {}".format(feedback_mode.x, feedback_mode.y, feedback_mode.z))
        # time.sleep(3)

        # feedback_mode.x = 0.0
        # feedback_mode.y = -2.0
        # feedback_mode.z = 0.0
        # self.feedback_pub.publish(feedback_mode)
        # print("x : {}, y : {}, z : {}".format(feedback_mode.x, feedback_mode.y, feedback_mode.z))
        # time.sleep(3)

        # feedback_mode.x = 0.0
        # feedback_mode.y = -4.0
        # feedback_mode.z = 0.0
        # self.feedback_pub.publish(feedback_mode)
        # print("x : {}, y : {}, z : {}".format(feedback_mode.x, feedback_mode.y, feedback_mode.z))
        # time.sleep(3)

        # feedback_mode.x = 0.0
        # feedback_mode.y = -8.0
        # feedback_mode.z = 0.0
        # self.feedback_pub.publish(feedback_mode)
        # print("x : {}, y : {}, z : {}".format(feedback_mode.x, feedback_mode.y, feedback_mode.z))
        # time.sleep(3)



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
