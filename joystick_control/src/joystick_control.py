#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Int32, Header, UInt16
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Joy, JoyFeedback
from joy_feedback_ros.msg import Rumble, Periodic

#User defined Macros for Analog Sticks on Xbox Controller
JOY_LX = 0
JOY_LY = 1
JOY_LB = 2
JOY_RX = 3
JOY_RY = 4
JOY_RB = 5
JOY_ARROW_LEFTRIGHT = 6
JOY_ARROW_UPDOWN = 7

# User defined Macros for Digital Buttons on Xbox Controller
JOY_A = 0
JOY_B = 1
JOY_X = 2
JOY_Y = 3
JOY_LT = 4
JOY_RT = 5
JOY_VIEW = 6
JOY_MENU = 7
JOY_XBOX = 8
JOY_LEFT_THUMB = 9
JOY_RIGHT_THUMB = 10

# Analog Stick Scale to reduce Speed (use value between 0.0 to 1.0)
STICK_SCALE = 1.0

#Joystick Analog Stick Tolerance
JOY_LX_DEADBAND = 0.5
JOY_LY_DEADBAND = 0.25
JOY_RX_DEADBAND = 0.25

# Joystick Button Defines
update_data = 0

print("Welcome!")

class JoystickControl():

    def __init__(self):
        # ROS
        # subscribe to joystick input
        self.subscriber = rospy.Subscriber("joy", Joy, self.get_joystick_callback)
        
        # publish the control command
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.publisher1 = rospy.Publisher("rumble", Rumble, queue_size=1)
        self.publisher2 = rospy.Publisher("periodic", Periodic, queue_size=1)
        self.publisher3 = rospy.Publisher("play", UInt16, queue_size=1)
        
        print("Mr. Bhushan Rane")

        self.flag = 1

    def get_joystick_callback(self,data):
        # Initialize published message
        twist = Twist()

        play = UInt16()
        play.data = 1
         
        vibration = Rumble()
        vibration.strong_magnitude = 18000
        vibration.weak_magnitude = 18000

        sticks = data.axes
        buttons = data.buttons

        if self.flag == 1:
            global update_data
            update_data = buttons
            self.flag = 0
        # print(sticks)
        # print(buttons)   

        # Map to command
        if sticks[JOY_LY] != 0:
            twist.linear.x = STICK_SCALE * sticks[JOY_LY]
            if (twist.linear.x >= -JOY_LY_DEADBAND and twist.linear.x <= JOY_LY_DEADBAND):
                twist.linear.x = 0
            #print(twist.linear.x)

        if sticks[JOY_LX] != 0:
            twist.angular.z = -(STICK_SCALE * sticks[JOY_LX])
            if (twist.angular.z >= -JOY_LX_DEADBAND and twist.angular.z <= JOY_LX_DEADBAND):
                twist.angular.z = 0
            #print(twist.angular.z)

        camera_movement = STICK_SCALE * sticks[JOY_RX]
        if (camera_movement >= -JOY_RX_DEADBAND and camera_movement <= JOY_RX_DEADBAND):
            camera_movement = 0

        # print(twist.linear.x)
        # print(twist.angular.z)
        # print(camera_movement)

        joy_button_A = (1<<0 & buttons[JOY_A])
        prev_joy_button_A = (1<<0 & update_data[JOY_A])
        joy_button_A_pressed = joy_button_A & ~prev_joy_button_A

        joy_button_B = (1<<0 & buttons[JOY_B])
        prev_joy_button_B = (1<<0 & update_data[JOY_B])
        joy_button_B_pressed = joy_button_B & ~prev_joy_button_B

        joy_button_X = (1<<0 & buttons[JOY_X])
        prev_joy_button_X = (1<<0 & update_data[JOY_X])
        joy_button_X_pressed = joy_button_X & ~prev_joy_button_X

        joy_button_Y = (1<<0 & buttons[JOY_Y])
        prev_joy_button_Y = (1<<0 & update_data[JOY_Y])
        joy_button_Y_pressed = joy_button_Y & ~prev_joy_button_Y

        if joy_button_A_pressed:
            vibration.strong_magnitude = 18000
            vibration.weak_magnitude = 18000
            self.publisher1.publish(vibration)  
            self.publisher3.publish(play)
            print("Button Pressed : A")
        elif joy_button_B_pressed:
            vibration.strong_magnitude = 10000
            vibration.weak_magnitude = 10000
            self.publisher1.publish(vibration)  
            self.publisher3.publish(play)
            print("Button Pressed : B")
        elif joy_button_X_pressed:
            vibration.strong_magnitude = 1000
            vibration.weak_magnitude = 1000
            self.publisher1.publish(vibration)  
            self.publisher3.publish(play)
            print("Button Pressed : X")
        elif joy_button_Y_pressed:
            vibration.strong_magnitude = 1500
            vibration.weak_magnitude = 1500
            self.publisher1.publish(vibration)  
            self.publisher3.publish(play)
            print("Button Pressed : Y")     

        update_data = buttons

        # rospy.loginfo("Send twist commad: Lx: ", twist.linear.x, ", Az: ", twist.angular.z)
        self.publisher.publish(twist)
        


if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('joy_control_node')
    joystick_control = JoystickControl()

    # Set rate and run
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()