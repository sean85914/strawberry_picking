#!/usr/bin/env python

import rospy
import serial
import sys
import select
from std_msgs.msg import Float64

pub_weight = rospy.Publisher('/weight', Float64, queue_size = 10)
	
if __name__ == '__main__':
	rospy.init_node('weight_serial_node', anonymous = False)
	port = rospy.get_param("~port", "/dev/ttyACM0")
	ard = serial.Serial(port, 38400)
	while True:
		try:
			# If there is an input, reset the scale
			if select.select([sys.stdin,], [], [], 1.0)[0]:
				# Wait one second
				x = sys.stdin.read(2)
				ard.write('r')
			else:
				while ard.inWaiting(): # To clear the buffer
					str = ard.readline()
				split_str = str.split(' ') # Split the string with space
				# Transmit format: "raw: data1 filtered: data2"
				if len(split_str) == 4: # To make sure not lose of data
					msg = Float64() 
					msg.data = float(split_str[3]) # Convert the filtered data to float
					pub_weight.publish(msg)
					rospy.loginfo(msg.data)
		except StopIteration:
			break
