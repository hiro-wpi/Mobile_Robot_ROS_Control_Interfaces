#!/usr/bin/env python

from re import S
import rospy
from std_msgs.msg import String, Float32, Int32, Header, UInt16, UInt8, Float64
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Joy, JoyFeedback, LaserScan
from joy_feedback_ros.msg import Rumble, Periodic
from time import time, sleep, strftime, time_ns

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
JOY_RY_DEADBAND = 0.25

COLLISION_CLEARANCE = 1

#print("Welcome!")

class JoystickControl():

    def __init__(self):
        # ROS
        # subscribe to joystick input
        self.subscriber = rospy.Subscriber("joy", Joy, self.get_joystick_callback)
        self.subscriber2 = rospy.Subscriber("base_scan", LaserScan, self.get_feedback)
        # publish the control command
        self.publisher = rospy.Publisher("base_controller/cmd_vel", Twist, queue_size=1)
        self.publisher1 = rospy.Publisher("rumble", Rumble, queue_size=1)
        self.publisher2 = rospy.Publisher("periodic", Periodic, queue_size=1)
        self.publisher3 = rospy.Publisher("play", UInt16, queue_size=1)
        
        self.camera_yaw = rospy.Publisher("main_cam_yaw_controller/command", Float64, queue_size=1)
        self.camera_pitch = rospy.Publisher("main_cam_pitch_controller/command", Float64, queue_size=1)
        print("Initialised Joystick")

        self.flag = 1
        self.rumble_flag = 0
        self.feedback_flag = 0
        self.vibration_data = 1000
        self.prev_vibration_data = 1000
        self.left_arrow_flag = 0
        self.right_arrow_flag = 0
        self.up_arrow_flag = 0
        self.down_arrow_flag = 0
        self.laser_range = []
        self.prev_now = 0
    def get_feedback(self, data):
        self.laser_range = data.ranges
        temp = self.laser_range[90:100]
        print(temp)
        # Center laser data start index : 82 and end index :110
        for i in temp:
            if i > 0.05 and i <= COLLISION_CLEARANCE:
                self.feedback_flag = 1
            else:
                self.feedback_flag = 0   


    def map(x, in_min, in_max, out_min, out_max):
        return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min


    def get_joystick_callback(self,data):
        # Initialize published message
        twist = Twist()
        camera_yaw_movement = Float64()
        camera_pitch_movement = Float64()
        play = UInt16()
        play.data = 0
         
        vibration = Rumble()
        
        sticks = data.axes
        buttons = data.buttons

        # One Time Flag to save current controller button state once.
        if self.flag == 1:
            global update_data, update_left_arrow, update_right_arrow, update_up_arrow, update_down_arrow
            update_data = buttons
            update_left_arrow = self.left_arrow_flag
            update_right_arrow = self.right_arrow_flag
            update_up_arrow = self.up_arrow_flag
            update_down_arrow = self.down_arrow_flag
            sleep(1)
            vibration.strong_magnitude = 16000
            vibration.weak_magnitude = 16000
            play.data = 0
            self.publisher1.publish(vibration)
            self.publisher3.publish(play)
            play.data = 1
            self.flag = 0
            print("CONTROLLER FORCE FEEDBACK INITIALIZATION SUCCESSFULLY COMPLETE")

        # Map to command

        # ((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min)
        if sticks[JOY_LY] != 0:
            twist.linear.x = STICK_SCALE * sticks[JOY_LY]
            if (twist.linear.x >= -JOY_LY_DEADBAND and twist.linear.x <= JOY_LY_DEADBAND):
                twist.linear.x = 0

        if sticks[JOY_LX] != 0:
            twist.angular.z = -(STICK_SCALE * sticks[JOY_LX])
            if (twist.angular.z >= -JOY_LX_DEADBAND and twist.angular.z <= JOY_LX_DEADBAND):
                twist.angular.z = 0

        camera_movement = STICK_SCALE * sticks[JOY_RX]
        if (camera_movement >= -JOY_RX_DEADBAND and camera_movement <= JOY_RX_DEADBAND):
            camera_movement = 0
        
        camera_yaw_movement = STICK_SCALE * sticks[JOY_RX]
        if (camera_yaw_movement >= -JOY_RX_DEADBAND and camera_yaw_movement <= JOY_RX_DEADBAND):
            camera_yaw_movement = 0

        camera_pitch_movement = STICK_SCALE * sticks[JOY_RY]
        if (camera_pitch_movement >= -JOY_RY_DEADBAND and camera_pitch_movement <= JOY_RY_DEADBAND):
            camera_pitch_movement = 0

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

        if (sticks[JOY_ARROW_LEFTRIGHT] == 0):
            self.left_arrow_flag = 0
            self.right_arrow_flag = 0
        elif (sticks[JOY_ARROW_LEFTRIGHT] == 1):
            self.left_arrow_flag = 1
            self.right_arrow_flag = 0
        elif (sticks[JOY_ARROW_LEFTRIGHT] == -1):
            self.left_arrow_flag = 0
            self.right_arrow_flag = 1

        joy_button_arrow_left = (1<<0 & self.left_arrow_flag)
        prev_joy_button_arrow_left = (1<<0 & (int)(update_left_arrow))
        joy_button_arrow_left_pressed = joy_button_arrow_left & ~prev_joy_button_arrow_left

        joy_button_arrow_right = (1<<0 & self.right_arrow_flag)
        prev_joy_button_arrow_right = (1<<0 & (int)(update_right_arrow))
        joy_button_arrow_right_pressed = joy_button_arrow_right & ~prev_joy_button_arrow_right

        if (sticks[JOY_ARROW_UPDOWN] == 0):
            self.up_arrow_flag = 0
            self.down_arrow_flag = 0
        elif (sticks[JOY_ARROW_UPDOWN] == 1):
            self.up_arrow_flag = 1
            self.down_arrow_flag = 0
        elif (sticks[JOY_ARROW_UPDOWN] == -1):
            self.up_arrow_flag = 0
            self.down_arrow_flag = 1

        joy_button_arrow_up = (1<<0 & self.up_arrow_flag)
        prev_joy_button_arrow_up = (1<<0 & (int)(update_up_arrow))
        joy_button_arrow_up_pressed = joy_button_arrow_up & ~prev_joy_button_arrow_up

        joy_button_arrow_down = (1<<0 & self.down_arrow_flag)
        prev_joy_button_arrow_down = (1<<0 & (int)(update_down_arrow))
        joy_button_arrow_down_pressed = joy_button_arrow_down & ~prev_joy_button_arrow_down

        if joy_button_A_pressed:
            print("Button Pressed : A")
        
        elif joy_button_B_pressed:
            vibration.strong_magnitude = 16000
            vibration.weak_magnitude = 16000
            play.data = 0
            self.publisher1.publish(vibration)
            self.publisher3.publish(play)     
            print("Button Pressed : B")
        
        elif joy_button_X_pressed:
            print("Button Pressed : X")
        
        elif joy_button_Y_pressed:
            print("Button Pressed : Y") 

        elif joy_button_arrow_left_pressed:
            print("Button Pressed : LEFT ARROW")

        elif joy_button_arrow_right_pressed:
            print("Button Pressed : RIGHT ARROW")

        elif joy_button_arrow_up_pressed:
            self.vibration_data = self.vibration_data + 1000
            print("Button Pressed : UP ARROW")

        elif joy_button_arrow_down_pressed:
            self.vibration_data = self.vibration_data - 1000
            print("Button Pressed : DOWN ARROW")
        
        print("Feedback_flag : ", self.feedback_flag)
        if self.feedback_flag:
            self.rumble_flag = 1

        print("TIME :: ", strftime("%S"))
        if (self.rumble_flag == 1):
            play.data = 1
            
            now = strftime("%S")
            
                
            if now != self.prev_now:
                self.publisher3.publish(play)
                self.rumble_flag = 0
                self.prev_now = now
        # if (self.vibration_data != self.prev_vibration_data):
        #     if(self.vibration_data < 0):
        #         self.vibration_data = 0
        #     elif (self.vibration_data > 18000):
        #         self.vibration_data = 18000 
        #     print("Vibration Magnitude:", self.vibration_data)
        #     vibration.strong_magnitude = self.vibration_data
        #     vibration.weak_magnitude = self.vibration_data
        #     self.rumble_flag = 0
        #     if (self.rumble_flag ==0):
        #         self.publisher1.publish(vibration)
        #         self.rumble_flag = 1
        #     if(self.rumble_flag == 1):
        #         self.publisher3.publish(play)
        #         self.rumble_flag = 2
        #     self.prev_vibration_data = self.vibration_data

        # now = rospy.get_rostime()
        # if(now.secs % 2):
        #     self.publisher1.publish(vibration)
        #     self.publisher3.publish(play)
        
        # if(self.feedback_flag == 1):
        #     now = rospy.get_rostime()
        #     if(now.secs % 2):
        #         self.publisher1.publish(vibration)
        #         self.publisher3.publish(play)
        #     if(now.secs % 5 == 4):
        #         self.feedback_flag = 2
             
        # BELOW LINES OF CODE ARE USED TO SAVE UPDATED BUTTON STATE
        # Add code if any more buttons are added but never delete - Bhushan Ashok Rane (barane@wpi.edu) 
        update_data = buttons
        update_left_arrow = self.left_arrow_flag
        update_right_arrow = self.right_arrow_flag
        update_up_arrow = self.up_arrow_flag
        update_down_arrow = self.down_arrow_flag

        # rospy.loginfo("Send twist command: Lx: ", twist.linear.x, ", Az: ", twist.angular.z)
        self.publisher.publish(twist)  

        #Publish Camera pitch and yaw
        #self.camera_pitch.publish(camera_pitch_movement)
        #self.camera_yaw.publish(camera_yaw_movement)  
        

if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('joy_control_node')
    joystick_control = JoystickControl()

    # Set rate and run
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()