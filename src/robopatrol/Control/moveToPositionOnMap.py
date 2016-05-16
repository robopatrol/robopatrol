#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
import move_base_msgs.msg


class MoveToPositionOnMap(object):

    def __init__(self):

	rospy.init_node('MoveToPosition', anonymous=False)

	'''
	Constructs a SimpleActionClient and opens connections to an ActionServer.
	The move_base package provides an implementation of an action.
	'''
	self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	
	'''
	Wait for the action server to report that it has come up and is ready to begin processing goals.
	Blocks until the action server connects to this client.
	'''
	rospy.loginfo('Waiting for the action server to come up.')
	self.client.wait_for_server()
	rospy.loginfo('The action server is online.')	


    '''
    @params 	int[] point, basically a 3d vector 	= POSIION
		int[] quaternion, a 4d vector		= ORIENTATION
		timeoutSeconds: max duration in seconds for robot to navigate to the goal
				navigation goal will be cancelled if time-out is reached
    '''
    def moveToPositionOnMap(self, point, quaternion, timeoutSeconds=30):

	result = -1

	if not rospy.is_shutdown():
	    self.client.cancel_goals_at_and_before_time(rospy.Time.now())
	
	goal = move_base_msgs.msg.MoveBaseGoal()

	# set coordinate frame
	goal.target_pose.header.frame_id = self._coordinateFrame

	# set time stamp
	goal.target_pose.header.stamp = rospy.Time.now()

	# set the pose (position + orientation)
	# position (3d point)
	goal.target_pose.pose.position.x = point[0]
	goal.target_pose.pose.position.y = point[1]
	goal.target_pose.pose.position.z = point[2]
	# orientation
	goal.target_pose.pose.orientation.x = quaternion[0]
	goal.target_pose.pose.orientation.y = quaternion[1]
	goal.target_pose.pose.orientation.z = quaternion[2]
	goal.target_pose.pose.orientation.w = quaternion[3]
	
	# send the navigation goal to the navigation stack
	if not rospy.is_shutdown():
	    self.client.send_goal(goal)

	countSeconds = 0
	finished = False
	while not finished and not rospy.is_shutdown():	    
	    if countSeconds > timeoutSeconds:
		finished = True
		self.client.cancel_goal()
 	    rospy.sleep(1)
	    count += 1
	    result = self.client.getState()

	rosply.loginfo(result)
	if result == GoalStatus.SUCCEEDED:
	    return True
	else:
	    return False


    def stop(self):
	self.client.cancel_goal()

