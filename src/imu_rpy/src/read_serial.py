#!/usr/bin/env python
import serial
import rospy
import tf
from nav_msgs.msg import Odometry
odom_br = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
ard = serial.Serial("/dev/ttyACM0", 9600)
def main():
	#r = rospy.Rate(10)
	while not rospy.is_shutdown():
		str = ard.readline()
		print str
		list = str.split(' ')
		if len(list) == 6:
			try:
				x = float(list[0])
				y = float(list[1])
				theta = float(list[2])
				v_x = float(list[3])
				v_y = float(list[4])
				omega = float(list[5])
			except ValueError:
				pass				
			#print x, y, theta, v_x, v_y, omega
			quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
			odom_br.sendTransform((x, y, 0),
						quaternion,
						rospy.Time.now(),
						"odom",
						"base_link")
			odom = Odometry()
			odom.header.stamp = rospy.Time.now()
			odom.header.frame_id = "odom"
			odom.pose.pose.position.x = x
			odom.pose.pose.position.y = y
			odom.pose.pose.position.z = 0
			odom.pose.pose.orientation.x = quaternion[0]
			odom.pose.pose.orientation.y = quaternion[1]
			odom.pose.pose.orientation.z = quaternion[2]
			odom.pose.pose.orientation.w = quaternion[3]
			odom.child_frame_id = "base_link"
			odom.twist.twist.linear.x = v_x
			odom.twist.twist.linear.y = v_y
			#odom.twist.twist.linear.z = 0
			#odom.twist.twist.angular.x = 0
			#odom.twist.twist.angular.y
			odom.twist.twist.angular.z = omega
			odom_pub.publish(odom)
		#r.sleep()
if __name__ == '__main__':
	rospy.init_node("odometry")
	main()
	rospy.spin()

