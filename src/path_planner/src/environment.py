#!/usr/bin/python3
import sys

import rospy

from gazebo_msgs.msg import ModelState

from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3

TABLE_HEIGHT = 1.045

def GModel(name, position, orientation = {'x': 0, 'y': 0, 'z': 0, 'w': 1}):
	return {
		'name': name,
		'position': position,
		'orientation': orientation
	}

class Environment:
	def __init__(self):
		self.models = [
			GModel('GreenBox', {
				'x': -0.186303,
				'y': 0.411113,
				'z': TABLE_HEIGHT,
			}),
			GModel('RedBox', {
				'x': -0.5163,
				'y': 0.4,
				'z': TABLE_HEIGHT,
			}, {
				'x': 0,
				'y': 0,
				'z': -0.168,
				'w': 1,
			}),
			GModel('BlueBox', {
				'x': -0.560,
				'y': 0.2759,
				'z': TABLE_HEIGHT,
			}),
			GModel('DepositBoxGreen', {
				'x': 0.463934,
				'y': 0.454994,
				'z': TABLE_HEIGHT,
			}),
			GModel('DepositBoxRed', {
				'x': 0.458589,
				'y': 0.239243,
				'z': TABLE_HEIGHT,
			}),
			GModel('DepositBoxBlue', {
				'x': 0.460276,
				'y': 0.022851,
				'z': TABLE_HEIGHT,
			}),
		]
		self.model_publisher = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10, latch=True)


	def reset(self):
		default_twist = Twist(Vector3(0, 0, 1), Vector3(0, 0, 0))

		for model in self.models:
			pose = Pose(
				Point(
					model['position']['x'],
					model['position']['y'],
					model['position']['z'],
				),
				Quaternion(
					model['orientation']['x'],
					model['orientation']['y'],
					model['orientation']['z'],
					model['orientation']['w'],
				)
			)
			self.model_publisher.publish(model['name'], pose, default_twist , 'world')


def menu():
	print('1) Reset environment')
	print('q) Exit')
	sys.stdout.flush()
	return input()


if __name__ == '__main__':
	rospy.init_node("env_controller", anonymous=True)
	env = Environment()

	while not rospy.is_shutdown():
		ans = menu()
		if(ans == '1'):
			env.reset()
		elif(ans == 'q'):
			sys.exit()

