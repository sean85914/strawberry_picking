#!/usr/bin/env python
# Read temperature data measured from DS18B20 through serial
# and publish as ROS topic 
# Editor: Sean, Lu
# Last update: 3/30, 2018
'''
    Update: add port as parameter (3/30, 2018)
'''
import rospy
import serial
from std_msgs.msg import Float64

temp_pub = rospy.Publisher('/temp/air_temp', Float64, queue_size = 10)

def main():
	while 1:
		res_str = ard.readline()
		res = float(res_str)
		msg = Float64()
		msg.data = res
		temp_pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('ard_serial', anonymous = False)
	port = rospy.get_param("~port", "/dev/ttyUSB0")
	# Remember to permit port first
	ard = serial.Serial(port, 9600)
	main()
	rospy.spin()
