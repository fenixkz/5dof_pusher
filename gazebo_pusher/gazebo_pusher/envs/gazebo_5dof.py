#!/usr/bin/env python3
from __future__ import print_function
import gym
import rospy
import tf
import roslaunch
import time
import numpy as np
import tf


from gym import utils, spaces
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from gym.utils import seeding
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetLinkState

def callback(data):
	angle = data.position

def rad2deg(ang):
    return ang*180/3.14

def deg2rad(ang):
    return ang*3.14159265359/180



class GazeboPlanar5DofEnv(gym.Env):
	def __init__(self):
		print("Created an environment")
		rospy.init_node('rl_gym')
		self.motor1_pub = rospy.Publisher('/robot/joint1_position_controller/command', Float64, queue_size=5)
		self.motor2_pub = rospy.Publisher('/robot/joint2_position_controller/command', Float64, queue_size=5)
		self.motor3_pub = rospy.Publisher('/robot/joint3_position_controller/command', Float64, queue_size=5)
		self.motor4_pub = rospy.Publisher('/robot/joint4_position_controller/command', Float64, queue_size=5)
		self.motor5_pub = rospy.Publisher('/robot/joint5_position_controller/command', Float64, queue_size=5)
		self.angle_step = 2
		self.action_space = spaces.Discrete(50) #F,L,R
		self.reward_range = (-np.inf, np.inf)
		quaternion = tf.transformations.quaternion_from_euler(0.000338, 0.003608, 0.000875)
		self.box = ModelState()
		self.box.model_name = 'robot1'
		self.box.pose.position.x = -2.87
		self.box.pose.position.y = 0
		# self.box.pose.position.z = 0.012142
		self.box.pose.position.z = 0
		self.initial_x = 5 + self.box.pose.position.x
		self.box.pose.orientation.x = quaternion[0]
		self.box.pose.orientation.y = quaternion[1]
		self.box.pose.orientation.z = quaternion[2]
		self.box.pose.orientation.w = quaternion[3]
		self.box.reference_frame = 'world'
		self.msg = [1.7564973394647119, -0.8303822610535567, -1.670635381665238, -0.8979926220318672, 1.642187184591668]
		self.pos_old = 5 + self.box.pose.position.x
		self.flag_1 = 0
		self.flag_2 = 0
		self.flag_3 = 0
		self.flag_4 = 0
		self._seed()



	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]


	def step(self, action):
		angles = None
		while angles is None:
			try:
				angles = rospy.wait_for_message('/robot/joint_states', JointState, timeout=5)
			except:
				pass
		rate = rospy.Rate(50)
		anglu = self.angle_step
		if action == 0: #Base motor left
			msg = angles.position[4] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[4] - msg) < 0.01):
					break
				rate.sleep()


		elif action == 1: #Base motor right
			msg = angles.position[4] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[4] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 2: #Second motor left
			msg = angles.position[1] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[1] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 3: #Second motor right
			msg = angles.position[1] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[1] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 4: #Third motor left
			msg = angles.position[2] - deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[2] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 5: #Third motor right
			msg = angles.position[2] + deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[2] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 6: #Fourth motor left
			msg = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor4_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[3] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 7: #Fourth motor right
			msg = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor4_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[3] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 8: #Fifth motor left
			msg = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor5_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print("!!")
				#print(msg)

				if (abs(s.position[0] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 9: #Fifth motor right
			msg = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor5_pub.publish(Float64(msg))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(s.position[0] - msg) < 0.01):
					break
				rate.sleep()

		elif action == 10: #1st left 2nd left
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[1] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor2_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[1] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 11: #1st left 2nd right
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[1] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor2_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[1] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 12: #1st right 2nd right
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[1] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor2_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[1] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 13: #1st right 2nd left
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[1] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor2_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[1] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 14: #1st left 3rd left
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[2] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				#print(msg)
				#print(s.position)
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 15: #1st left 3rd right
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[2] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 16: #1st right 3rd right
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[2] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 17: #1st right 3rd left
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[2] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 18: #1st left 4th left
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 19: #1st left 4th right
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 20: #1st right 4th right
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 21: #1st right 4th left
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 22: #1st left 5th left
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[0] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 23: #1st left 5th right
			msg1 = angles.position[4] - deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 24: #1st right 5th right
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 25: #1st right 5th left
			msg1 = angles.position[4] + deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor1_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[4] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg1) < 0.01):
					break
				rate.sleep()

		elif action == 26: #2nd left 3rd left
			msg1 = angles.position[1] - deg2rad(anglu)
			msg2 = angles.position[2] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 27: #2nd left 3rd right
			msg1 = angles.position[1] - deg2rad(anglu)
			msg2 = angles.position[2] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 28: #2nd right 3rd right
			msg1 = angles.position[1] + deg2rad(anglu)
			msg2 = angles.position[2] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 29: #2nd right 3rd left
			msg1 = angles.position[1] + deg2rad(anglu)
			msg2 = angles.position[2] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor3_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[2] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 30: #2nd left 4th left
			msg1 = angles.position[1] - deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 31: #2nd left 4th right
			msg1 = angles.position[1] - deg2rad(anglu)
			msg2 = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 32: #2nd right 4th right
			msg1 = angles.position[1] + deg2rad(anglu)
			msg2 = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 33: #2nd right 4th left
			msg1 = angles.position[1] + deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 34: #2nd left 5th left
			msg1 = angles.position[1] - deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 35: #2nd left 5th right
			msg1 = angles.position[1] - deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 36: #2nd right 5th right
			msg1 = angles.position[1] + deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg2) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 37: #2nd right 5th left
			msg1 = angles.position[1] + deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor2_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[1] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 38: #3rd left 4th left
			msg1 = angles.position[2] - deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 39: #3rd left 4th right
			msg1 = angles.position[2] - deg2rad(anglu)
			msg2 = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg1) < 0.01):
					break
				rate.sleep()

		elif action == 40: #3rd right 4th right
			msg1 = angles.position[2] + deg2rad(anglu)
			msg2 = angles.position[3] + deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 41: #3rd right 4th left
			msg1 = angles.position[2] + deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor4_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 42: #3rd left 5th left
			msg1 = angles.position[2] - deg2rad(anglu)
			msg2 = angles.position[3] - deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[3] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 43: #3rd left 5th right
			msg1 = angles.position[2] - deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg1) < 0.01):
					break
				rate.sleep()

		elif action == 44: #3rd right 5th right
			msg1 = angles.position[2] + deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 45: #3rd right 5th left
			msg1 = angles.position[2] + deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor3_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[2] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 46: #4th left 5th left
			msg1 = angles.position[3] - deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor4_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[3] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 47: #4th left 5th right
			msg1 = angles.position[3] - deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor4_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[3] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 48: #4th right 5th right
			msg1 = angles.position[3] + deg2rad(anglu)
			msg2 = angles.position[0] + deg2rad(anglu)
			while True:
				self.motor4_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[3] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		elif action == 49: #4th right 5th left
			msg1 = angles.position[3] + deg2rad(anglu)
			msg2 = angles.position[0] - deg2rad(anglu)
			while True:
				self.motor4_pub.publish(Float64(msg1))
				s = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				if (abs(s.position[3] - msg1) < 0.01):
					break
				self.motor5_pub.publish(Float64(msg2))
				k = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
				#print(msg)
				#print(s.position)
				if (abs(k.position[0] - msg2) < 0.01):
					break
				rate.sleep()

		angles = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
		sensor1 = sensor2 = sensor3 = sensor4 = sensor5 = None
		while  sensor1 is None and sensor2 is None and sensor3 is None and sensor4 is None and sensor5 is None:
			try:
				sensor1 = rospy.wait_for_message('/robot_sensor1', ContactsState, timeout = 5)
				sensor2 = rospy.wait_for_message('/robot_sensor2', ContactsState, timeout = 5)
				sensor3 = rospy.wait_for_message('/robot_sensor3', ContactsState, timeout = 5)
				sensor4 = rospy.wait_for_message('/robot_sensor4', ContactsState, timeout = 5)
				sensor5 = rospy.wait_for_message('/robot_sensor5', ContactsState, timeout = 5)
			except:
				pass

		done = False
		# reward = 0

		try:
			aa = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
			a = aa("box","world")
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

		for x in angles.position:
			if abs(abs(x)-2) < 0.08:
				done = True
				reward = -1
				print("State is out of range")
		contact = [sensor1.states != [], sensor2.states != [], sensor3.states != [], sensor4.states != [], sensor5.states != []]
		contact = list(map(int, contact))
		# if (abs(a.link_state.pose.position.x - self.pos_old) > 0.01):
		# 	reward = 0.2
		# # if (a.link_state.pose.position.x > 2):
		# # 	reward = 0.5
		# if (a.link_state.pose.position.x > 3):
		# 	reward = 0.5
		# if (sum(contact) == 0):
		# 	done = True
		# 	reward = -1
		# elif (a.link_state.pose.position.x > 4):
		# 	done = True
		# 	reward = 1
		reward = self.calculate_reward(a.link_state.pose.position.x + 5, contact)
		if (reward == -1 or reward == 1):
			done = True
		# print("Old position: "+str(self.pos_old)+" ; now position is: "+str(a.link_state.pose.position.x))
		# self.pos_old = a.link_state.pose.position.x
		# print(type(angles.position))
		# print(contact)
		state = angles.position + tuple(contact)
		# state = self.states2obs(angles.position)
		return state, reward, done, a.link_state.pose.position.x + 5


	def reset(self):
        # Resets the state of the environment and returns an initial observation.
		self.flag_1 = 0
		self.flag_2 = 0
		self.flag_3 = 0
		self.flag_4 = 0
		while True:
			self.motor1_pub.publish(Float64(self.msg[4]))
			self.motor2_pub.publish(Float64(self.msg[1]))
			self.motor3_pub.publish(Float64(self.msg[2]))
			self.motor4_pub.publish(Float64(self.msg[3]))
			self.motor5_pub.publish(Float64(self.msg[0]))
			angles = rospy.wait_for_message('/robot/joint_states', JointState, timeout = 5)
			if (abs(angles.position[4] - self.msg[4]) < 0.01 and abs(angles.position[1] - self.msg[1]) < 0.01 and \
			    abs(angles.position[2] - self.msg[2]) < 0.01 and abs(angles.position[3] - self.msg[3]) < 0.01 and abs(angles.position[0] - self.msg[0]) < 0.01):
				break

		time.sleep(0.5)
		angles = None
		while angles is None:
			try:
				angles = rospy.wait_for_message('/robot/joint_states', JointState, timeout=5)
			except:
				pass
		try:
			set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
			resp = set_state(self.box)
		except rospy.ServiceException as e:
			print("Service call failed: %s" % e)

		sensor1 = sensor2 = sensor3 = sensor4 = sensor5 = None
		while  sensor1 is None and sensor2 is None and sensor3 is None and sensor4 is None and sensor5 is None:
			try:
				sensor1 = rospy.wait_for_message('/robot_sensor1', ContactsState, timeout = 5)
				sensor2 = rospy.wait_for_message('/robot_sensor2', ContactsState, timeout = 5)
				sensor3 = rospy.wait_for_message('/robot_sensor3', ContactsState, timeout = 5)
				sensor4 = rospy.wait_for_message('/robot_sensor4', ContactsState, timeout = 5)
				sensor5 = rospy.wait_for_message('/robot_sensor5', ContactsState, timeout = 5)
			except:
				pass
		contact = [sensor1.states != [], sensor2.states != [], sensor3.states != [], sensor4.states != [], sensor5.states != []]
		contact = list(map(int, contact))
		state = angles.position + tuple(contact)
		return state

	def calculate_reward(self, x, contact):
		reward = 0
		init_x = 2.13
		x = x - init_x
		if (sum(contact) == 0):
			return -1
		if (x > 1.8):
			return 1
		# dx = x - self.initial_x
		if (x > 0.3 and not self.flag_1):
			reward = 0.1
			self.flag_1 = 1
		if (x > 0.7 and not self.flag_2):
			reward = 0.2
			self.flag_2 = 1
		if (x > 1 and not self.flag_3):
			reward = 0.3
			self.flag_3 = 1
		if (x > 1.3 and not self.flag_4):
			reward = 0.4
			self.flag_4 = 1

		return reward
