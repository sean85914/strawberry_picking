#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations as tfm
#from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
	
class MoveItCartesianPath:
	def __init__(self):
		rospy.init_node("grasp_object", anonymous=False)
		rospy.loginfo("Starting node grasp_object")
		rospy.on_shutdown(self.cleanup)	
		self.sub_waypoint_number = rospy.Subscriber("/ws1/way_point_number", Int32, self.cb_waypoint, queue_size = 1)
		self.pub_waypoint_feedback = rospy.Publisher("/ws1/way_point_feedback", Int32, queue_size = 1)
		self.index_to_go = 0
		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)
		# Initialize the move group for the ur5_arm
		self.arm = moveit_commander.MoveGroupCommander('manipulator')
		# Get the name of the end-effector link
		end_effector_link = self.arm.get_end_effector_link()
		# Set the reference frame for pose targets
		reference_frame = "/base_link"
		# Set the ur5_arm reference frame accordingly
		self.arm.set_pose_reference_frame(reference_frame)
		# Allow replanning to increase the odds of a solution
		self.arm.allow_replanning(True)
		# Allow some leeway in position (meters) and orientation (radians)
		self.arm.set_goal_position_tolerance(0.005)
		self.arm.set_goal_orientation_tolerance(0.05)
		self.arm.set_max_velocity_scaling_factor(0.01) 		
		# Open file and build the 2D list
		self.file_name = rospy.get_param("~file_name")
		file_ = '/home/joinet/catkin_ws/src/ur_modern_driver/' + self.file_name
		file = open(file_)
		self.waypoint_list = []
		line_list = []
		while 1:
			line = file.readline()
			if line == '':
				break
			else:
				line_split = line.split(' ')
				for i in range(7):
					line_list.append(float(line_split[i]))
				self.waypoint_list.append(line_list)
				line_list = []
        	file.close()      
       	 	# Get the current pose so we can add it as a waypoint
        	self.start_pose = self.arm.get_current_pose(end_effector_link).pose

	def cb_waypoint(self, msg):
		self.index_to_go = msg.data+1
		# Move UR-5 to original pose
		waypoints = []
        	#waypoints.append(self.start_pose)
        	wpose = deepcopy(self.start_pose)
        	wpose.position.x = self.waypoint_list[0][0]
        	wpose.position.y = self.waypoint_list[0][1]
        	wpose.position.z = self.waypoint_list[0][2]
        	wpose.orientation.x = self.waypoint_list[0][3]
        	wpose.orientation.y = self.waypoint_list[0][4]	
        	wpose.orientation.z = self.waypoint_list[0][5]
        	wpose.orientation.w = self.waypoint_list[0][6]
        	waypoints.append(deepcopy(wpose))
	
        	fraction = 0.0
        	maxtries = 100
        	attempts = 0

        	self.arm.set_start_state_to_current_state()

        	# Plan the Cartesian path connecting the waypoints
        	while fraction < 1.0 and attempts < maxtries:
			(plan, fraction) = self.arm.compute_cartesian_path (waypoints, 0.01, 0.0, True)
			# Increment the number of attempts
			attempts += 1
			# Print out a progress message
			if attempts % 10 == 0:
				rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
            	# If we have a complete plan, execute the trajectory
        	if fraction == 1.0:
            		rospy.loginfo("Path computed successfully. Moving the arm.")
            		self.arm.execute(plan)
            		rospy.loginfo("Path execution complete.")
        	else:
            		rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
		rospy.sleep(1) # Hold on one second
		# Move to the waypoint 
		# Initialize the waypoints list
        	waypoints = []
        	# Set the first waypoint to be the starting pose
        	# Append the pose to the waypoints list
        	#waypoints.append(self.start_pose)
        	# Go to the waypoint 
        	wpose = deepcopy(self.start_pose)
        	wpose.position.x = self.waypoint_list[self.index_to_go][0]
       		wpose.position.y = self.waypoint_list[self.index_to_go][1]
        	wpose.position.z = self.waypoint_list[self.index_to_go][2]
        	wpose.orientation.x = self.waypoint_list[self.index_to_go][3]
        	wpose.orientation.y = self.waypoint_list[self.index_to_go][4]
        	wpose.orientation.z = self.waypoint_list[self.index_to_go][5]
        	wpose.orientation.w = self.waypoint_list[self.index_to_go][6]
        	waypoints.append(deepcopy(wpose))   
        	fraction = 0.0
        	maxtries = 100
       		attempts = 0
        	# Set the internal state to the current state
        	self.arm.set_start_state_to_current_state()
        	# Plan the Cartesian path connecting the waypoints
        	while fraction < 1.0 and attempts < maxtries:
			(plan, fraction) = self.arm.compute_cartesian_path (waypoints, 0.01, 0.0, True)
            		# Increment the number of attempts
            		attempts += 1
            		# Print out a progress message
            		if attempts % 10 == 0:
                		rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
            			# If we have a complete plan, execute the trajectory
		if fraction == 1.0:
            		rospy.loginfo("Path computed successfully. Moving the arm.")
            		self.arm.execute(plan)
            		rospy.loginfo("Path execution complete.")
        	else:
            		rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")
		# Reached
		feedback = Int32()
		feedback.data = self.index_to_go
		self.pub_waypoint_feedback.publish(feedback)
		rospy.sleep(0.5)
	def cleanup(self):
		rospy.loginfo("Stopping the robot")

		# Stop any current arm movement
		self.arm.stop()

		#Shut down MoveIt! cleanly
		rospy.loginfo("Shutting down Moveit!")
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)
    


if __name__ == "__main__":
	try:
		MoveItCartesianPath()
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down MoveItCartesianPath node."
