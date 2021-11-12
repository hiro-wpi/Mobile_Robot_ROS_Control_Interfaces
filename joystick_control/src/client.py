#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


import socket
import time
import os
import json

def callback(msg):
	#print('x value',msg.linear.x)
	s=socket.socket()

	#host=socket.gethostname() #server hostname

	host="192.168.0.101"

	port=22022 #same as server

	#s.send('e'.encode()) 

	s.connect((host,port))

	#fileToSend = open(“ToSend.txt”,”r”)

	content1 = "base.pole.sit"

	contentnavi = "navigate.drive"

	content2 = "navigate.drive,{'throttle' : 1.0, 'turn' : 0.0}"



	data_f = {'throttle' : msg.linear.x, 'turn' : msg.angular.z} 

	data_b = {'throttle' : 1.0, 'turn' : 0.0} #move backward

	content = "camera.enable,{ 'template': 'screen' }"



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