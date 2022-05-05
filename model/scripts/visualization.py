#!/usr/bin/env python
# Created by qian at 5/4/22

# Description:

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf

import numpy as np
import json

data_path = '/home/qian/Documents/study/6_832_project/src/model/data/'


def add_obstacle(x, y, radius, id):
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "/world"
	marker.id = id
	marker.type = Marker.CYLINDER
	marker.action = 0
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = radius * 2
	marker.scale.y = radius * 2
	marker.scale.z = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	marker.lifetime = rospy.Duration.from_sec(0)

	return marker


def add_control_point(point, id):
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "/world"
	marker.id = id
	marker.type = Marker.SPHERE
	marker.action = 0
	marker.pose.position.x = point[0]
	marker.pose.position.y = point[1]
	marker.pose.position.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.1
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	marker.lifetime = rospy.Duration.from_sec(0)

	return marker


def add_ref_points(points, id):
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "/world"
	marker.id = id
	marker.type = Marker.SPHERE_LIST
	marker.action = 0
	marker.pose.position.x = 0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.1
	marker.scale.y = 0.1
	marker.scale.z = 0.1
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	for point in points:
		pnt = Point()
		pnt.x = point[0]
		pnt.y = point[1]
		pnt.z = 0.0
		marker.points.append(pnt)
	marker.lifetime = rospy.Duration.from_sec(0)

	return marker


def publish_world(path):
	markers = MarkerArray()

	# markers.markers.append(add_obstacle(1, -2, 1, 0))
	# markers.markers.append(add_obstacle(2, 1, 1, 1))
	# markers.markers.append(add_obstacle(5, 0.5, 1.5, 2))
	# markers.markers.append(add_obstacle(9.5, -0.5, 1.5, 3))
	with open(path, 'r') as f:
		world_str = f.readlines()[0]
		world_str = world_str.replace("'", "\"")
		world_dict = json.loads(world_str)

		if 'ref_points' in world_dict.keys():
			markers.markers.append(add_ref_points(world_dict['ref_points'], 0))

		enum = 1
		if 'control_points' in world_dict.keys():
			for point in world_dict['control_points']:
				markers.markers.append(add_control_point(point, enum))
				enum += 1

		if 'obstacle' in world_dict.keys():
			for obstacle in world_dict['obstacle']:
				markers.markers.append(add_obstacle(obstacle[0], obstacle[1], obstacle[2], enum))
				enum += 1

	return markers


def publish_solutions(br, step_state):
	# state = JointState()
	# state.header.stamp = rospy.get_rostime()
	# state.name = ['base_to_wheel_x', 'base_to_wheel_y', 'base_to_wheel_r', 'wheel_to_rod']
	# state.position = [step_state[0], step_state[1], step_state[2], step_state[5]]
	# pub.publish(state)
	br.sendTransform((step_state[0], step_state[1], 0),
		             tf.transformations.quaternion_from_euler(0, 0, step_state[2] + np.pi/2),
		             rospy.Time.now(),
		             'wheel_base',
		             'base_link')
	br.sendTransform((0.2, 0.1, 0),
		             tf.transformations.quaternion_from_euler(step_state[5] + np.pi/2, 0, 0),
		             rospy.Time.now(),
		             'rod',
		             'wheel_base')


if __name__ == '__main__':
	rospy.init_node('data_converter')

	states_opt = np.load(data_path + "state0.npy")
	markers = publish_world(data_path + 'world0.txt')

	world_pub = rospy.Publisher('world_obstacles', MarkerArray, queue_size=10)
	# joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	br = tf.TransformBroadcaster()

	r = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			for i in range(states_opt.shape[0]):
				world_pub.publish(markers)
				publish_solutions(br, states_opt[i, :])
				r.sleep()
			# rospy.sleep()
	except KeyboardInterrupt:
		exit()
