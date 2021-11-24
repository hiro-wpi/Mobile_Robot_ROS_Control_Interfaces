#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
#import falcon_control

import socket
import time
import os
import json

def callback(msg):
    print('x value',msg.linear.x)
    s=socket.socket()

	#host=socket.gethostname() #server hostname

    host="192.168.0.101"

    port=22022 #same as server

	#s.send('e'.encode()) 

    s.connect((host,port))

    contentnavi = "navigate.drive"
    data_f = {'throttle' : msg.linear.x, 'turn' : msg.angular.z}

	#camera_control  = tilt.move{ "speed":0}

    contenta = ["navigate.enable",""]

    packet = { 'c': contentnavi }
    if data_f is not None: packet['d'] = data_f
    jsonString = json.dumps(packet)
	#self.sock.send(jsonString.encode('utf-8'))
    print(jsonString)
    s.send(jsonString.encode('utf-8'))


def interface_listener():
	# Need to make something which will help classify from which topic the msgs are comming for, for different interfaces.
	rospy.init_node('double3_sub')
	sub = rospy.Subscriber('/cmd_vel',Twist,callback)

	rospy.spin()

if __name__ == '__main__':
	interface_listener()
