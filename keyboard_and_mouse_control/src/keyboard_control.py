#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pynput import keyboard
from geometry_msgs.msg import Twist
    
class KeyboardControl():
    def __init__(self):
        self.pub = rospy.Publisher('keyboard_double3', Twist, queue_size=1)



    def key_map(self, key):
        
        # Forward motion (Keyboard keys : W and up-arrow)  -- {navigate, {throttle:'1', turn : '0'}}
        # Backward motion (Keyboard keys : s and down-arrow)  -- {navigate, {throttle:'-1', turn : '0'}}
        # Turn right (Keyboard keys : D and right-arrow)  -- {navigate, {throttle:'0', turn : '0.5'}}
        # Left right (Keyboard keys : D and left-arrow)  -- {navigate, {throttle:'0', turn : '-0.5'}}
        # 
        global msg
        msg = Twist()
        
        rospy.init_node('keyboard_double3_publisher', anonymous=True)
        rate = rospy.Rate(1) # 1hz
        #print('key pressed : ', current_key)
        #if key == '':
        #    msg.linear.x = 0.0
        #    pub.publish(msg)
        #if key == 'space':
        #if key == '':
        #    msg.linear.x = 0.0
        #    pub.publish(msg) 
        if key == 'w':
            msg.linear.x = 1.0
            msg.angular.z = 0.0 
            self.pub.publish(msg)
        if key == 's':
            msg.linear.x = -1.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
        if key == 'd':
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.pub.publish(msg)
        if key == 'a':
            msg.linear.x = 0.0
            msg.angular.z = -0.5
            self.pub.publish(msg) 
        
            
    


        #print('key pressed : ', key)

    def on_press(self, key):
        global current_key
        try:
            if key == keyboard.Key.shift:
                print('shift key!')
                current_key = ''
            else:
                current_key = key.char
            #print('current key pressed', current_key)
        except AttributeError:
            print('key {0} pressed'.format(key))
            current_key = ''


    def on_release(self, key):
        global current_key
        current_key = ''
        #if key == keyboard.Key.esc:
        #    return False
        if key == keyboard.Key.ctrl:
            return False
        else:
            current_key = ''

    if __name__ == '__main__':
        
        current_key = ''

        print('press CTRL  to finish the programm')

        # starts keyboard listener
        listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)
        listener.start()

        while listener.is_alive():
            
            #print('key pressed : ', current_key)

            key_map(current_key)