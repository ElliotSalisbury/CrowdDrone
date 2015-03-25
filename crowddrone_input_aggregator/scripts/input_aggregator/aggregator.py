#!/usr/bin/env python
import rospy
from threading import Thread, Timer
from abc import ABCMeta, abstractmethod
from geometry_msgs.msg import Twist
from copy import deepcopy

class Aggregator(Thread):
	__metaclass__ = ABCMeta

	def __init__(self,connectedClients, connectedLock, topicname='cmd_vel'):
		Thread.__init__(self)
		self.connectedClients=connectedClients
		self.connectedLock=connectedLock
		self.publisher = rospy.Publisher(topicname, Twist, queue_size=0)
		self.oldTwist = Twist()
		self.initialize()
		self.active = True
		self.stopTimer = None

	def getConnectedClients(self):
		# with self.connectedLock:
		# 	connectedSnapshot = deepcopy(self.connectedClients)
		# 	return connectedSnapshot
		return self.connectedClients

	def stop(self):
		self.active = False

	def initialize(self):
		pass

	def publish(self, twist):
		#check new twist is different to old twist
		if self.active and twist and not self.twistIsEmpty(twist):
			#only worth publishing if the twist is different
			# otherwise it'll carry on till timer runs out
			if not self.twistIsEqual(twist, self.oldTwist):
				rospy.loginfo("aggregator parent publish: %s", twist)
				self.publisher.publish(twist)
				self.oldTwist = twist

			#stop the current timer and start a new count down for this action to end
			if self.stopTimer:
				self.stopTimer.shutdown()
			self.stopTimer = rospy.Timer(rospy.Duration(1), self.publishStop, oneshot=True)

	def publishStop(self, timerEvent):
		twist = Twist()
		rospy.loginfo("aggregator parent publish: %s", twist)
		self.publisher.publish(twist)
		self.oldTwist = twist

	@abstractmethod
	def run(self):
		pass

	def twistIsEmpty(self, twist):
		return (twist.linear.x == 0 and
						twist.linear.y == 0 and
						twist.linear.z == 0 and
						twist.angular.x == 0 and
						twist.angular.y == 0 and
						twist.angular.z == 0)
	def twistIsEqual(self, twist, oldTwist):
		return (twist.linear.x == oldTwist.linear.x and
						twist.linear.y == oldTwist.linear.y and
						twist.linear.z == oldTwist.linear.z and
						twist.angular.x == oldTwist.angular.x and
						twist.angular.y == oldTwist.angular.y and
						twist.angular.z == oldTwist.angular.z)

	def notifyClientUpdated(self, topic):
		pass

	def getCandidates(self):
		#define the different candidates
		forward = Twist()
		forward.linear.x = 1
		backward = Twist()
		backward.linear.x = -1
		left = Twist()
		left.angular.z = 1
		right = Twist()
		right.angular.z = -1
		noaction = Twist()

		return [forward, backward, left, right, noaction]
	def rankVote(self, twist):
		#define the different candidates
		forward = Twist()
		forward.linear.x = 1
		backward = Twist()
		backward.linear.x = -1
		left = Twist()
		left.angular.z = 1
		right = Twist()
		right.angular.z = -1
		noaction = Twist()

		#rank for left
		if(twist.angular.z > 0):
			return [3, 3, 0, 4, 2.9]
		#rank for right
		if(twist.angular.z < 0):
			return [3, 3, 4, 0, 2.9]
		#rank for forward
		if(twist.linear.x > 0):
			return [0, 4, 3, 3, 2.9]
		#rank for backward
		if(twist.linear.x < 0):
			return [4, 0, 3, 3, 2.9]
		#rank for noaction
		return [4, 4, 4, 4, 3.9]

	def getLinearCandidates(self):
		#define the different candidates
		forward = Twist()
		forward.linear.x = 1
		backward = Twist()
		backward.linear.x = -1
		noaction = Twist()

		return [forward, noaction, backward]
	def rankLinearVote(self, twist):

		forward = Twist()
		forward.linear.x = 1
		backward = Twist()
		backward.linear.x = -1
		noaction = Twist()

		#rank for forward
		if(twist.linear.x > 0):
			return [0, 0.9, 2]
		#rank for backward
		elif(twist.linear.x < 0):
			return [2, 0.9, 0]
		#rank for noaction
		else:
			return [2, 1.9, 2]

	def getAngularCandidates(self):
		#define the different candidates
		left = Twist()
		left.angular.z = 1
		right = Twist()
		right.angular.z = -1
		noaction = Twist()

		return [left, noaction, right]
	def rankAngularVote(self, twist):
		#rank for left
		if(twist.angular.z > 0):
			return [0, 0.9, 2]
		#rank for right
		elif(twist.angular.z < 0):
			return [2, 0.9, 0]
		#rank for noaction
		else:
			return [2, 1.9, 2]
