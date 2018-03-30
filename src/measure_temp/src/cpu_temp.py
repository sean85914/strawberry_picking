#!/usr/bin/env python
# Get RPi temperature from commandline and publish as a ROS topic 
# Run commandline in a python script reference
# https://unix.stackexchange.com/questions/190495/how-to-execute-a-bash-command-in-a-python-script?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa

# Editor: Sean, Lu
# Last update: 3/30, 2018

import subprocess
import rospy
from std_msgs.msg import Float64
# Publisher
pub_temp = rospy.Publisher('/temp/pi_cpu_temp', Float64, queue_size = 10)

def get_temp(event):
	res_str = subprocess.check_output(['/opt/vc/bin/vcgencmd', 'measure_temp'])
	res = float(res_str[5:9])
	print res
	msg = Float64()
	msg.data = res
	pub_temp.publish(msg)

if __name__ == '__main__':
	rospy.init_node('cpu_temp', anonymous = False)
	rospy.Timer(rospy.Duration.from_sec(1), get_temp)
	rospy.spin()
	
	
