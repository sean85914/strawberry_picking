#!/usr/bin/env python
# Input: Strawberry tf
# Output: Strawberrt position w.r.t base
import rospy
import tf
from geometry_msgs.msg import Pose

if __name__ == '__main__':
	rospy.init_node('strawberry_position')
	listener = tf.TransformListener()
	
	strawberry_position = rospy.Publisher('my_pcl_tutorial/strawberry_position', Pose)
	rate = rospy.Rate(10) # ?
	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('base_link', 'Strawberry', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		strawberry_pose = Pose()
		strawberry_pose.position.x = trans[0]
		strawberry_pose.position.y = trans[1]
		strawberry_pose.position.z = trans[2]
		strawberry_pose.orientation.x = rot[0]
		strawberry_pose.orientation.y = rot[1]
		strawberry_pose.orientation.z = rot[2]
		strawberry_pose.orientation.w = rot[3]
		print trans # show the result at terminal
		strawberry_position.publish()
		
		rate.sleep()
		
