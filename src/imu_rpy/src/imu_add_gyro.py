#!/usr/bin/python
import rospy
import time
import math
import numpy
import tf
from Adafruit_LSM303 import Adafruit_LSM303
from Gyro_L3GD20 import Gyro_L3GD20

# 


class AdafruitIMU(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.compass_accel = Adafruit_LSM303()
		self.gyro = Gyro_L3GD20()
		self.pub_timestep = self.setupParam("~pub_timestep", 0.02)
		self.br = tf.TransformBroadcaster()
		self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep), self.publish)
		# Const
		self.DEG_TO_RAD = 0.00174533
		# Bool
		self.first_time = True
		# RPY
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		rospy.loginfo("[%s] Initialized..." %(self.node_name))
	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s" %(self.node_name, param_name, value))
		return value
	def publish(self, event):
		compass_accel = self.compass_accel.read()
		compass = compass_accel[1]
		accel = compass_accel[0]
		gyro = self.gyro.read()
		acc_x = accel[0]
		acc_y = accel[1]
		acc_z = accel[2]
		gyro_rad = [0, 0, 0]
		for i in range(3):
			gyro_rad[i] = gyro[0][i] * self.DEG_TO_RAD
		roll = math.atan2(acc_y, acc_z)
		pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z))
		mag_x_tilt = compass[0] * math.cos(pitch) + compass[1] * math.sin(pitch)
		mag_y_tilt = compass[0] * math.sin(roll) * math.sin(pitch) + compass[2] * math.cos(roll) - compass[1] * math.sin(roll) * math.cos(pitch)
		yaw = math.atan2(mag_y_tilt, mag_x_tilt)
		self.roll  = 0.02 * roll  + (self.roll + self.pub_timestep * gyro_rad[0]) * 0.98
		self.pitch = 0.02 * pitch + (self.pitch + self.pub_timestep * gyro_rad[1]) * 0.98
		self.yaw   = 0.02 * yaw   + (self.yaw + self.pub_timestep * gyro_rad[2]) * 0.98
		# compass[0] > mag_x
		# compass[1] > mag_z
		# compass[2] > mag_y
		#yaw = math.atan2(compass[2], compass[0])
		self.br.sendTransform((0, 0, 0),
					tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw),
					rospy.Time.now(),
					"IMU",
					"world")
		rospy.loginfo("[%s] Roll: %f, Pitch: %f, Yaw: %f" %(self.node_name, self.roll, self.pitch, self.yaw))
	def on_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." %(self.node_name))
if __name__ == "__main__":
	rospy.init_node("Adafruit_IMU_test", anonymous = False)
	Adafruit_IMU = AdafruitIMU()
	rospy.on_shutdown(Adafruit_IMU.on_shutdown)
	rospy.spin()
