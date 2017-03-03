#!/usr/bin/env python

import rospy
from agile_grasp.srv import *
from geometry_msgs.msg import *
from agile_grasp.msg import Grasps

data = list()

def callback(msg):
	global data
	if len(msg.grasps) < 200:
		data = msg.grasps
		print "Found ", len(data), " possible grasping poses."

def handle_grasp_pose(req):
	n_sub = rospy.Subscriber("/find_grasps/grasps", Grasps, callback)
	rospy.sleep(22)
	return grasp_poseResponse(data)

def grasp_pose_server():
	rospy.init_node('grasp_pose_server')
	s = rospy.Service('grasp_pose', grasp_pose, handle_grasp_pose)
	rospy.loginfo("Ready to subscribe to feasible grasp pose")
	rospy.spin()

if __name__ == '__main__':
	grasp_pose_server()