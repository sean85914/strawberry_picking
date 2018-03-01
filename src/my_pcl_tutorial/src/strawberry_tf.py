#!/usr/bin/env python
import rospy
import tf
from visualization_msgs.msg import Marker
def cb_marker(msg):
	br = tf.TransformBroadcaster()
	br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
			tf.transformations.quaternion_from_euler(0, 0, 0),
			rospy.Time.now(),
			"Strawberry",
			"camera_rgb_optical_frame")
if __name__ == "__main__":
	rospy.init_node("strawberry_tf_broadcaster")
	rospy.Subscriber("/my_pcl_tutorial/marker/stem_to_cut", Marker, cb_marker)
	rospy.spin()
