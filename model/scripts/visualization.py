#!/usr/bin/env python
# Created by qian at 5/4/22

# Description:

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import tf

import numpy as np

data_path = '/home/qian/Documents/study/6_832_project/src/model/data/state0.npy'


def add_marker(x, y, radius, id):
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


def publish_world(pub):
	markers = MarkerArray()

	markers.markers.append(add_marker(1, -2, 1, 0))
	markers.markers.append(add_marker(2, 1, 1, 1))
	markers.markers.append(add_marker(5, 0.5, 1.5, 2))
	markers.markers.append(add_marker(9.5, -0.5, 1.5, 3))

	pub.publish(markers)


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

	states_opt = np.load(data_path)

	world_pub = rospy.Publisher('world_obstacles', MarkerArray, queue_size=10)
	# joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	br = tf.TransformBroadcaster()

	r = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			for i in range(states_opt.shape[0]):
				publish_world(world_pub)
				publish_solutions(br, states_opt[i, :])
				r.sleep()
			# rospy.sleep()
	except KeyboardInterrupt:
		exit()
