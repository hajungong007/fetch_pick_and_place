#!/usr/bin/env python

import rospy
import actionlib
import copy
import numpy
import sys
import tf

import moveit_commander
import moveit_msgs.msg

from moveit_python import PlanningSceneInterface

from geometry_msgs.msg import *
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from agile_grasp.msg import Grasps
from std_msgs.msg import String, Int32
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import *

from agile_grasp.srv import *

class FollowTrajectoryClient(object):

	def __init__(self, name, joint_names):
		self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
			                                       FollowJointTrajectoryAction)
		rospy.loginfo("Waiting for %s..." % name)
		self.client.wait_for_server()
		self.joint_names = joint_names

	def move_to(self, positions, duration=5.0):
		if len(self.joint_names) != len(positions):
			rospy.loginfo("Invalid trajectory position!!!!!")
			return False
		trajectory = JointTrajectory()
		trajectory.joint_names = self.joint_names
		trajectory.points.append(JointTrajectoryPoint())
		trajectory.points[0].positions = positions
		trajectory.points[0].velocities = [0.0 for _ in positions]
		trajectory.points[0].accelerations = [0.0 for _ in positions]
		trajectory.points[0].time_from_start = rospy.Duration(duration)
		follow_goal = FollowJointTrajectoryGoal()
		follow_goal.trajectory = trajectory

		self.client.send_goal(follow_goal)
		self.client.wait_for_result()


class PointHeadClient(object):

	def __init__(self):
		self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
		rospy.loginfo("Waiting for head_controller...")
		self.client.wait_for_server()

	def look_at(self, x, y, z, frame, duration = 1.0):
		goal = PointHeadGoal()
		goal.target.header.stamp = rospy.Time.now()
		goal.target.header.frame_id = frame
		goal.target.point.x = x
		goal.target.point.y = y
		goal.target.point.z = z
		goal.min_duration = rospy.Duration(duration)
		self.client.send_goal(goal)
		self.client.wait_for_result()


class GripperCommandClient(object):

	def __init__(self):
		self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
		rospy.loginfo("Waiting for gripper_controller...")
		self.client.wait_for_server()

	def open(self):
		move = GripperCommandGoal()
		move.command.position = 0.10
		move.command.max_effort = 0.0
		rospy.loginfo("Command gripper to open ...")
		self.client.send_goal(move)
		self.client.wait_for_result()
		if (self.client.get_result().reached_goal == True):
			rospy.loginfo("Successfully placed object ...")
		else:
			rospy.loginfo("Failed placed object ...")
		
	def close(self):
		squeeze = GripperCommandGoal()
		squeeze.command.position = 0.0
		squeeze.command.max_effort = 50.0
		rospy.loginfo("Command gripper to close ...")
		self.client.send_goal(squeeze)
		self.client.wait_for_result()
		if (self.client.get_result().reached_goal == True):
			rospy.loginfo("Failed to grasp object ...")
			return False
		else:
			rospy.loginfo("Successfully grasped object ...")
			return True


class GraspingClient(GripperCommandClient):

	def __init__(self):
		# initialize GripperCommandClient
		GripperCommandClient.__init__(self)
		# initialize Moveit Scene
		self.scene = PlanningSceneInterface("base_link")
		self.scene2 = moveit_commander.PlanningSceneInterface()
		find_topic = "basic_grasping_perception/find_objects"
		rospy.loginfo("Waiting for %s..." % find_topic)
		self.find_client = actionlib.SimpleActionClient(find_topic,
														FindGraspableObjectsAction)
		self.find_client.wait_for_server()
		# creat target pose publisher
		self.marker_pub = rospy.Publisher("/TargetMarker", MarkerArray, queue_size=1)
		# creat basket pos publisher
		self.marker_pub2 = rospy.Publisher("/basket", MarkerArray, queue_size=1)
		# creat publisher for requesting grasp pose
		self.request_cloud_client = rospy.Publisher("/request_cloud", Int32, queue_size=1)
		# instantiate a RobotCommander
		self.robot = moveit_commander.RobotCommander()
		# instantiate a MoveGroupCommander
		self.group = moveit_commander.MoveGroupCommander("arm")
		# basket's parameters 
		self.basket_pos = 'left'
		self.basket_found = False
		self.basket_pos_x = 0
		self.basket_pos_y = 0
		self.basket_search_t = 0

	def updateScene(self):
		# detect objects
		goal = FindGraspableObjectsGoal()
		goal.plan_grasps = False
		self.find_client.send_goal(goal)
		self.find_client.wait_for_result(rospy.Duration(5.0))
		find_result = self.find_client.get_result()

		#remove original objects
		self.clear_scene()
		# all the objects in scene
		self.objects = list()
		# height is used for grasping selection
		self.height	= -1
		# object number
		object_num = -1

		# add support surface to scene
		for obj in find_result.support_surfaces:
			# extend surface to floor
			h = obj.primitive_poses[0].position.z
			if (h + obj.primitives[0].dimensions[2] / 2.0) > self.height:
				self.height = h + obj.primitives[0].dimensions[2] / 2.0
				self.table = obj 

			obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0] + 0.02,
											obj.primitives[0].dimensions[1] + 0.02,
											obj.primitives[0].dimensions[2] + h - 0.02]
			obj.primitive_poses[0].position.z += -h / 2.0
			# add to scene
			self.scene.addSolidPrimitive(obj.name,
										 obj.primitives[0],
										 obj.primitive_poses[0],
										 wait = False)
		# add objects Solid to scene
		for obj in find_result.objects:
			object_num += 1
			obj.object.name = "object%d"%object_num

			self.scene.addSolidPrimitive(obj.object.name,
							   obj.object.primitives[0],
							   obj.object.primitive_poses[0],
							   wait = False)

			if obj.object.primitive_poses[0].position.z > self.height:
				self.objects.append([obj, numpy.array([obj.object.primitive_poses[0].position.x,
												 	  obj.object.primitive_poses[0].position.y,
												 	  obj.object.primitive_poses[0].position.z]), obj.object.primitive_poses[0].position.x])
			
			# localize basket
			if obj.object.primitive_poses[0].position.y > self.table.primitive_poses[0].position.y + self.table.primitives[0].dimensions[1] / 2.0 + 0.1 or \
				obj.object.primitive_poses[0].position.y < self.table.primitive_poses[0].position.y - self.table.primitives[0].dimensions[1] / 2.0 - 0.1:
				if obj.object.primitives[0].dimensions[1] > 0.20 and obj.object.primitive_poses[0].position.z > 0.3 and not self.basket_found:
					if self.basket_search_t == 1:
						self.basket_pos_x = obj.object.primitive_poses[0].position.x
						self.basket_pos_y = obj.object.primitive_poses[0].position.y
						self.basket_search_t += 1
					elif self.basket_search_t == 2:
						if self.basket_pos_x - obj.object.primitive_poses[0].position.x < 0.04 and \
							self.basket_pos_y - obj.object.primitive_poses[0].position.y < 0.04:
								self.basket_found = True
								if obj.object.primitive_poses[0].position.y < 0:
									self.basket_pos = 'right'
								self.basket = obj
								self.marker_pub2.publish(self.basket_marker())
								rospy.loginfo("Basket is found at %s..." % self.basket_pos)
						else:
							self.basket_search_t -= 1

		if self.basket_search_t == 0:
			self.basket_search_t += 1

		self.scene.waitForSync()

	def clear_scene(self):
		for name in self.scene.getKnownCollisionObjects():
			self.scene.removeCollisionObject(name, False)
		for name in self.scene.getKnownAttachedObjects():
			self.scene.removeAttachedObject(name, False)
		self.scene.waitForSync()

	def clear_target_object(self, pose):
		if len(self.objects) == 1:
			self.scene.removeCollisionObject(self.objects[0][0].object.name)
			self.tname = self.objects[0][0].object.name
		else:
			dist = float('inf')
			vec = numpy.array([pose[0].center.x + 0.33 * pose[0].approach.x, pose[0].center.y + 0.33 * pose[0].approach.y, pose[0].center.z + 0.33 * pose[0].approach.z])
			for obj in self.objects:
				temp = numpy.linalg.norm(obj[1] - vec)
				if temp < dist:
					dist = temp
					self.tname = obj[0].object.name
			self.scene.removeCollisionObject(self.tname, False)

	def readd_target_object(self):
		for obj in self.objects:
			if obj[0].object.name == self.tname:
				self.scene.addSolidPrimitive(obj[0].object.name,
											   obj[0].object.primitives[0],
											   obj[0].object.primitive_poses[0],
											   wait = False)

	def request_grasp_poses(self, mode = None):
		# possible grasp poses
		self.grasp_pose = list()

		rospy.wait_for_service('grasp_pose')
		res = rospy.ServiceProxy('grasp_pose', grasp_pose)

		grasps_obtained = False
		while not grasps_obtained:
			try:
				self.grasp_pose[:] = []
				rospy.loginfo("Try to find feasible grasp poses ...")
				# request PointCloud data for agile grasp package
				self.request_cloud_client.publish(1)
				grasps = res()
				if mode == 'cluster':
					self.pose_selection_cluster(grasps.grasps)
				else:
					self.pose_selection_general(grasps.grasps)
				if len(self.grasp_pose) > 0:
					self.TargetGraspPoses, self.Rots = self.creat_target_grasp_pose(self.grasp_pose)
					if self.TargetGraspPoses != None:
						grasps_obtained = True
			except rospy.ServiceException, e:
				rospy.loginfo("Service call failed: %s" %e)
				continue

	def pose_selection_general(self, poses):
		for pose in poses:
			if pose.approach.z <= 0.0 and \
				pose.width.data <= 0.06:
				z = numpy.array([0, 0, 1])
				y = numpy.array([0, 1, 0])
				vec1 = -1.0 * numpy.array([pose.approach.x, pose.approach.y, pose.approach.z])
				if pose.axis.z < 0:
					vec2 = -1.0 * numpy.array([pose.axis.x, pose.axis.y, pose.axis.z])
				else:
					vec2 = numpy.array([pose.axis.x, pose.axis.y, pose.axis.z])
				temp_dot1 = numpy.dot(z, vec1)
				temp_dot2 = numpy.dot(z, vec2)
				if (temp_dot1 < 0.2 and pose.approach.x >= 0 and pose.width.data < 0.05 and temp_dot2 > 0.95 and pose.center.z > self.height + 0.07) or \
					temp_dot1 > 0.9 and pose.center.z < self.height + 0.06 :
					self.grasp_pose.append([pose, pose.center.z, temp_dot1])
		self.grasp_pose.sort(key=lambda object:object[1])
		self.grasp_pose.reverse()
		
	def pose_selection_cluster(self, poses):	
		for pose in poses:
			if pose.approach.z < 0.0 and \
				pose.width.data < 0.06:
				z = numpy.array([0, 0, 1])
				vec = -1.0 * numpy.array([pose.approach.x, pose.approach.y, pose.approach.z])
				temp_dot = numpy.dot(z, vec)
				if temp_dot > 0.9:
					self.grasp_pose.append([pose, pose.center.z])
		self.grasp_pose.sort(key=lambda object:object[1])
		self.grasp_pose.reverse()
								
	def plan_and_pick(self, mode = None):
		rospy.loginfo("%d valid poses was found ..." % len(self.TargetGraspPoses))
		for i in range(0, len(self.TargetGraspPoses)):
			self.group.clear_pose_targets()
			# publish target grasping pose
			self.marker_pub.publish(self.target_pose_marker(self.TargetGraspPoses[i][0], self.Rots[i]))
			self.group.set_pose_target(self.TargetGraspPoses[i][0])
			num = 0
			res = False
			while not res and num < 2:
				num += 1
				rospy.loginfo("%d Planning to pick ..." %num)
				self.group.set_planner_id('KPIECEConfigDefault')
				self.group.set_max_velocity_scaling_factor(0.4)
				plan1 = self.group.plan()
				res = self.group.execute(plan1)
			if res:
				if mode == None:
					# remove target object from MoveIt scene
					self.clear_target_object(self.grasp_pose[self.TargetGraspPoses[i][1]])
					rospy.sleep(2.0)
				else:
					for obj in self.objects:
						self.scene.removeCollisionObject(obj[0].object.name, False)
			    #generate waypoints to approach the target object
				TargetWayPoints = self.pick_waypoints(self.grasp_pose[self.TargetGraspPoses[i][1]])
				(plan2, fraction) = self.group.compute_cartesian_path(TargetWayPoints, 0.001, 0.0)
				#print fraction	
				if fraction == 1.0:
					rospy.loginfo("Start to approach object ...")
					self.group.execute(plan2)
					rospy.sleep(1.0)
					rospy.loginfo("Try to grasp the object ...")
					res = self.close()
					rospy.sleep(1.0)
					if res:
						return True
					else:
						self.ready_to_grasp()
						self.open()
						self.updateScene()
						continue
				if mode == None:
					self.readd_target_object()
				else:
					for obj in self.objects:
						self.scene.addSolidPrimitive(obj[0].object.name,
											   obj[0].object.primitives[0],
											   obj[0].object.primitive_poses[0],
											   wait = False)
				self.ready_to_grasp()
				self.open()
		return False

	def plan_and_place(self, mode = None):
		TargetWayPoints = self.place_waypoints()
		(plan2, fraction) = self.group.compute_cartesian_path(TargetWayPoints, 0.001, 0.0)
		#print fraction
		if fraction == 1.0:
			rospy.loginfo("Lifting object ...")
			self.group.execute(plan2)
			if mode == None:
				self.attach_object()
				rospy.sleep(1.0)
			place_res = self.place_to_fixed_position(self.basket_pos)
			
			if place_res:
				rospy.loginfo("Successfully placed the object ...")
			else:
				rospy.loginfo("Failed to place the object")
		else:
			rospy.loginfo("Failed to place the object")

	def creat_target_grasp_pose(self, poses):
		pose_idx = -1
		valid_poses = list()
		valid_rots = list()
		for pose in poses:
			pose_idx += 1
			R = numpy.eye(4)
			R[0:3,0] = [pose[0].approach.x, pose[0].approach.y, pose[0].approach.z]		
			R[0:3,2] = [pose[0].axis.x, pose[0].axis.y, pose[0].axis.z]
			R[0:3,1] = numpy.cross(R[0:3,2], R[0:3,0])
			detR = numpy.linalg.det(R)
			if detR < 0.99997:
				continue
			pose[0].center.x = pose[0].center.x - 0.33 * pose[0].approach.x
			pose[0].center.y = pose[0].center.y - 0.33 * pose[0].approach.y
			pose[0].center.z = pose[0].center.z - 0.33 * pose[0].approach.z
			quat = tf.transformations.quaternion_from_matrix(R)
			point = Pose()
			point.position.x = pose[0].center.x 
			point.position.y = pose[0].center.y
			point.position.z = pose[0].center.z 
			point.orientation.x = quat[0]
			point.orientation.y = quat[1]
			point.orientation.z = quat[2]
			point.orientation.w = quat[3]
			valid_poses.append([point, pose_idx])
			valid_rots.append(R)
		if len(valid_poses) == 0:
			return None, None
		else:
			return valid_poses, valid_rots

	def pick_waypoints(self, pose):
		if pose[2] < 0.2:
			print pose[2]
			dist = [0.03, 0.06, 0.09, 0.12, 0.14]
		else:
			dist = [0.03, 0.06, 0.09, 0.125]
		pick_waypoints = list()
		pick_waypoints.append(self.group.get_current_pose().pose)
		for j in range(0, len(dist)):
			point = copy.deepcopy(pick_waypoints[0])
			point.position.x += dist[j] * pose[0].approach.x
			point.position.y += dist[j] * pose[0].approach.y
			point.position.z += dist[j] * pose[0].approach.z
			pick_waypoints.append(point)
		return pick_waypoints

	def place_waypoints(self):
		dist = [0.03, 0.06, 0.09, 0.12]
		place_waypoints = list()
		place_waypoints.append(self.group.get_current_pose().pose)
		for d in dist:
			wpose = copy.deepcopy(place_waypoints[0])
			wpose.position.z += d
			place_waypoints.append(wpose)
		return place_waypoints

	def target_pose_marker(self, pos, rot):
		markers = MarkerArray()
		for j in range(0,5):
			marker = Marker()
			marker.type = Marker.ARROW
			marker.id = j
			marker.header.frame_id = "base_link"
			marker.header.stamp = rospy.get_rostime()
			marker.lifetime = rospy.Duration.from_sec(120.0)
			marker.action = Marker.ADD
			marker.scale.x = 0.01
			marker.scale.y = 0.01
			marker.scale.z = 0.01
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.color.a = 1.0			
			p = Point()
			q = Point()			
			p.x = pos.position.x + (0.21 + 0.125) * rot[0,0]
			p.y = pos.position.y + (0.21 + 0.125) * rot[1,0]
			p.z = pos.position.z + (0.21 + 0.125) * rot[2,0]
			if j == 0: # left finger
				p.x = p.x - 0.05 * rot[0,1]
				p.y = p.y - 0.05 * rot[1,1]
				p.z = p.z - 0.05 * rot[2,1]
				q.x = p.x - 0.08 * rot[0,0]
				q.y = p.y - 0.08 * rot[1,0]
				q.z = p.z - 0.08 * rot[2,0]
			elif j == 1: # right finger
				p.x = p.x + 0.05 * rot[0,1]
				p.y = p.y + 0.05 * rot[1,1]
				p.z = p.z + 0.05 * rot[2,1]
				q.x = p.x - 0.08 * rot[0,0]
				q.y = p.y - 0.08 * rot[1,0]
				q.z = p.z - 0.08 * rot[2,0]
			elif j == 2: # base
				p.x = p.x - 0.05 * rot[0,1] - 0.08 * rot[0,0]
				p.y = p.y - 0.05 * rot[1,1] - 0.08 * rot[1,0]
				p.z = p.z - 0.05 * rot[2,1] - 0.08 * rot[2,0]
				q.x = p.x + 0.10 * rot[0,1]
				q.y = p.y + 0.10 * rot[1,1]
				q.z = p.z + 0.10 * rot[2,1]
			elif j == 3: # approach
				p.x = p.x - 0.08 * rot[0,0]
				p.y = p.y - 0.08 * rot[1,0]
				p.z = p.z - 0.08 * rot[2,0]
				q.x = p.x - 0.08 * rot[0,0]
				q.y = p.y - 0.08 * rot[1,0]
				q.z = p.z - 0.08 * rot[2,0]
			else : # axis
			    q.x = p.x + 0.15 * rot[0,2]
			    q.y = p.y + 0.15 * rot[1,2]
			    q.z = p.z + 0.15 * rot[2,2]  
			    marker.color.r = 0.0
			    marker.color.g = 0.0
			    marker.color.b = 1.0
			    marker.color.a = 1.0			
			marker.points.append(p)
			marker.points.append(q)
			markers.markers.append(marker)

		return markers

	def basket_marker(self):
		markers = MarkerArray()
		for j in range(0,4):
			marker = Marker()
			marker.type = Marker.ARROW
			marker.id = j
			marker.header.frame_id = "base_link"
			marker.header.stamp = rospy.get_rostime()
			marker.lifetime = rospy.Duration.from_sec(120.0)
			marker.action = Marker.ADD
			marker.scale.x = 0.01
			marker.scale.y = 0.01
			marker.scale.z = 0.01
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.color.a = 1.0			
			p = Point()
			q = Point()
			p.x = self.basket.object.primitive_poses[0].position.x
			p.y = self.basket.object.primitive_poses[0].position.y
			p.z = self.basket.object.primitive_poses[0].position.z + self.basket.object.primitives[0].dimensions[2] / 2.0
			q.z = p.z
			if j == 0:
				p.x = p.x + self.basket.object.primitives[0].dimensions[0] / 2.0
				p.y = p.y + self.basket.object.primitives[0].dimensions[1] / 2.0
				q.x = p.x 
				q.y = p.y - self.basket.object.primitives[0].dimensions[1]
			elif j == 1:
				p.x = p.x + self.basket.object.primitives[0].dimensions[0] / 2.0
				p.y = p.y - self.basket.object.primitives[0].dimensions[1] / 2.0
				q.x = p.x - self.basket.object.primitives[0].dimensions[0]
				q.y = p.y
			elif j == 2:
				p.x = p.x - self.basket.object.primitives[0].dimensions[0] / 2.0
				p.y = p.y - self.basket.object.primitives[0].dimensions[1] / 2.0
				q.x = p.x 
				q.y = p.y + self.basket.object.primitives[0].dimensions[1]
			else:
				p.x = p.x - self.basket.object.primitives[0].dimensions[0] / 2.0
				p.y = p.y + self.basket.object.primitives[0].dimensions[1] / 2.0
				q.x = p.x + self.basket.object.primitives[0].dimensions[0]
				q.y = p.y
			marker.points.append(p)
			marker.points.append(q)
			markers.markers.append(marker)
		return markers

	def ready_to_grasp(self):
		self.group.clear_pose_targets()
		#stow_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		stow_pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
		self.group.set_joint_value_target(stow_pose)
		res = False
		while not res:
			rospy.loginfo("Planning for moving to stow pose...")
			self.group.set_max_velocity_scaling_factor(0.4)
			plan = self.group.plan()
			rospy.sleep(1)
			res = self.group.execute(plan)

	def place_to_fixed_position(self, position):
		self.group.clear_pose_targets()
		if position == "left":
			pose = [0.8646, -0.4970, -0.0002, 0.7099, -0.0461, 1.3065, 0.0024]
		else:
			pose = [-0.8646, -0.4970, -0.0002, 0.7099, -0.0461, 1.3065, 0.0024]
		self.group.set_joint_value_target(pose)
		res = False
		count = 0
		while not res and count < 3:
			rospy.loginfo("Planing to place object...")
			self.group.set_max_velocity_scaling_factor(0.4)
			self.group.set_planner_id('KPIECEConfigDefault')
			plan = self.group.plan()
			res = self.group.execute(plan)
			count += 1
		return res

	def tuck(self):
		self.group.clear_pose_targets()
		stow_pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
		self.group.set_joint_value_target(stow_pose)
		res = False
		while not res:
			rospy.loginfo("Planing tuck ...")
			self.group.set_max_velocity_scaling_factor(0.4)
			plan = self.group.plan()
			rospy.sleep(1)
			res = self.group.execute(plan)

	def continue_or_quit(self, s):
		s = raw_input(s)
		#input 'q' to shutdown
		if s == 'q':
			self.scene.removeAttachedObject('box', False)
			self.scene.removeCollisionObject('box', False)
			self.open()
			self.readd_target_object()
			rospy.sleep(1.0)
			self.ready_to_grasp() 
			exit(0)

	def attach_object(self):
		listener = tf.TransformListener()
		rate = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			try:
				(trans,quat) = listener.lookupTransform('gripper_link', 'base_link', rospy.Time(0))
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		rot = tf.transformations.quaternion_matrix(quat)
		rot[0:3, 3] = trans

		for obj in self.objects:
			if obj[0].object.name == self.tname:
				if len(obj[0].object.primitives[0].dimensions) < 3:
					old_pos = numpy.array([obj[1][0], obj[1][1], (obj[1][2] + 0.12 - obj[0].object.primitives[0].dimensions[1] / 2.0), 1])
				else:
					old_pos = numpy.array([obj[1][0], obj[1][1], (obj[1][2] + 0.12 - obj[0].object.primitives[0].dimensions[2] / 2.0), 1])
				new_pos = numpy.dot(rot, old_pos)
				old_quat = numpy.array([obj[0].object.primitive_poses[0].orientation.x,
										obj[0].object.primitive_poses[0].orientation.y,
										obj[0].object.primitive_poses[0].orientation.z,
										obj[0].object.primitive_poses[0].orientation.w])
				old_ori = tf.transformations.quaternion_matrix(old_quat)
				new_ori = numpy.dot(rot, old_ori)
				new_quat = tf.transformations.quaternion_from_matrix(new_ori)
				point = PoseStamped()
				point.header.frame_id = 'gripper_link'
				point.header.stamp = rospy.get_rostime()
				point.pose.position.x = new_pos[0]
				point.pose.position.y = new_pos[1]
				point.pose.position.z = new_pos[2]
				point.pose.orientation.x = new_quat[0]
				point.pose.orientation.y = new_quat[1]
				point.pose.orientation.z = new_quat[2]
				point.pose.orientation.w = new_quat[3]
				if len(obj[0].object.primitives[0].dimensions) < 3:
					self.scene2.attach_box('gripper_link', 'box', point, (obj[0].object.primitives[0].dimensions[1],
																			obj[0].object.primitives[0].dimensions[1],
																			obj[0].object.primitives[0].dimensions[0] / 2.0),
																			['l_gripper_finger_link', 'r_gripper_finger_link'])
				else:
					self.scene2.attach_box('gripper_link', 'box', point, (obj[0].object.primitives[0].dimensions[0] - 0.01,
																			obj[0].object.primitives[0].dimensions[1] - 0.01,
																			obj[0].object.primitives[0].dimensions[2] / 2.3),
																			['l_gripper_finger_link', 'r_gripper_finger_link'])

        
if __name__ == '__main__':
	# initialize ROS and MoveIt
	rospy.init_node("fetch_grasping")
	moveit_commander.roscpp_initialize(sys.argv)

	# Setting up task mode:
	mode = None
	if len(sys.argv) == 2:
		mode = sys.argv[1]

	# setting up clients
	torso_client = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
	head_client1 = PointHeadClient()
	head_client2 = FollowTrajectoryClient("head_controller", ["head_pan_joint", "head_tilt_joint"])
	grasping_client = GraspingClient()

	# raising the torso
	rospy.loginfo("Raising torso ...")
	torso_client.move_to([0.35])

	# moving head
	rospy.loginfo("Looking at ...")
	head_client1.look_at(1.0, 0.0, 0.0, "base_link")

	# updating scence
	rospy.loginfo("Updating scene ...")
	grasping_client.updateScene()

	# prepare to grasp 
	rospy.loginfo("Move to stow pose ...")
	grasping_client.ready_to_grasp()
	
	while not rospy.is_shutdown():
		# check if there is object left
		if len(grasping_client.objects) == 0:
			rospy.loginfo("There is no object to be picked ...")
			break
		rospy.loginfo("%d objects are on the table ..." % len(grasping_client.objects))


		#rospy.loginfo("Searching basket location ...")
		#count = 0
		#while not grasping_client.basket_found:
		#	if count % 2 == 0:
		#		head_client2.move_to([0.22, 1.027])
		#	else:
		#		head_client2.move_to([-0.32, 1.027])
		#	rospy.sleep(0.5)
		#	grasping_client.updateScene()
		#	rospy.sleep(4.0)
		#	grasping_client.updateScene()
		#	count += 1

		# look at table and update scene
		head_client1.look_at(1.0, 0.0, 0.0, "base_link")
		rospy.loginfo("Updating scene ...")
		grasping_client.updateScene()
		

		pick_res = False
		place_res = False
		while not pick_res or not place_res:
			# subscribe to agile_grasp package
			grasping_client.request_grasp_poses(mode)

			# plan and pick object
			pick_res = grasping_client.plan_and_pick()
			
			if not pick_res :
				rospy.loginfo("Back to ready to pick pose ...")
				grasping_client.ready_to_grasp()
				grasping_client.open()
				grasping_client.updateScene()
				continue
				
			rospy.loginfo("Start to plan and place or stop ...")
			# plan and place object
			place_res = grasping_client.plan_and_place(mode)

			# place the object
			grasping_client.open()

			# remove attached object
			grasping_client.scene.removeAttachedObject('box')
			grasping_client.scene.removeCollisionObject('box')

			# return to ready to pick pose
			grasping_client.ready_to_grasp()
			break
			
		grasping_client.__init__()
		#rospy.sleep(30)
		#grasping_client.updateScene()



