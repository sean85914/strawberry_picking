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
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
    
class MoveItCartesianPath:
    def __init__(self):
        rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)
	# File name
	self.file_name = rospy.get_param("~file_name")
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
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
	file_ = '/home/joinet/catkin_ws/src/ur_modern_driver/' + self.file_name
        file = open(file_, 'w+')
        # Get the current pose so we can add it as a waypoint
        while 1:
            start_pose = self.arm.get_current_pose(end_effector_link).pose

            print start_pose
            orientation_list = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w] 
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            print roll, pitch, yaw
            quat = quaternion_from_euler (roll, pitch, yaw)
            print quat
        
            s = raw_input('waitkey: w to write, e to end')
            if s == 'w':
                print 'write pose'
                file.write(str(start_pose.position.x) + ' ' +str(start_pose.position.y) + ' ' +str(start_pose.position.z) + ' ' +str(start_pose.orientation.x) + ' ' + str(start_pose.orientation.y) + ' ' + str(start_pose.orientation.z) + ' ' + str(start_pose.orientation.w) + '\n');
            if s == 'e':
                file.close()
                break


        
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
    except KeyboardInterrupt:
        print "Shutting down MoveItCartesianPath node."
