#!/usr/bin/python
import rospy
import time
import math
import numpy
import tf
from Adafruit_LSM303 import Adafruit_LSM303
from Gyro_L3GD20 import Gyro_L3GD20

# Calculate yaw offset in 1 second after start up


class AdafruitIMU(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.compass_accel = Adafruit_LSM303()
		self.gyro = Gyro_L3GD20()
		# Time stamp: default is 0.02 s
		self.pub_timestep = self.setupParam("~pub_timestep", 0.02)
		# Tf broadcaster
		self.br = tf.TransformBroadcaster()
		# Timer
		self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep), self.publish)
		# Counter
		self.counter = 0
		# Yaw offset
		self.yaw_offset = 0
		# Set to True after counter reach 50
		self.state = None 
		# Log init completed
		rospy.loginfo("[%s] Initialized..." %(self.node_name))
	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s" %(self.node_name, param_name, value))
		return value
	def publish(self, event):
		# Read LSM303 data
		compass_accel = self.compass_accel.read()
		# compass[0]: x, compass[1]: 'z', compass[2]: y
		compass = compass_accel[1]
		# accel[0]: x, accel[1]: y, accel[2]: z
		accel = compass_accel[0]
		# Read L3GD20 data
		gyro = self.gyro.read()
		# gyro[0][0]: x, gyro[0][1]: y, gyro[0][2]
		acc_x = accel[0]
		acc_y = accel[1]
		acc_z = accel[2]
		roll = math.atan2(acc_y, acc_z)
		pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z))
		mag_x_tilt = compass[0] * math.cos(pitch) + compass[1] * math.sin(pitch)
		mag_y_tilt = compass[0] * math.sin(roll) * math.sin(pitch) + compass[2] * math.cos(roll) - compass[1] * math.sin(roll) * math.cos(pitch)
		yaw = math.atan2(mag_y_tilt, mag_x_tilt)
		if self.counter < 50: # 1 sec
			rospy.loginfo("Initializing... %d" %self.counter)
			self.yaw_offset += yaw
			self.counter += 1
		else:
			if self.state is None:
				self.state = True
				self.yaw_offset = self.yaw_offset / self.counter
			yaw = self.yaw_offset - yaw
		# Send tf if state is true
		# 'world' and 'imu' frame have the same origin but different orinetation
		if self.state is True:
			self.br.sendTransform((0, 0, 0),
						tf.transformations.quaternion_from_euler(roll, pitch, yaw),
						rospy.Time.now(),
						"IMU",
						"world")
			rospy.loginfo("[%s] Roll: %f, Pitch: %f, Yaw: %f" %(self.node_name, roll, pitch, yaw))
	def on_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %(self.node_name))
if __name__ == "__main__":
	rospy.init_node("Adafruit_IMU_test", anonymous = False)
	Adafruit_IMU = AdafruitIMU()
	rospy.on_shutdown(Adafruit_IMU.on_shutdown)
	rospy.spin()
