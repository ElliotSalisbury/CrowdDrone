#!/usr/bin/env python
import rospy
import math
from aggregator import Aggregator
from geometry_msgs.msg import Twist
from copy import deepcopy

class LeaderAggregator(Aggregator):
	def initialize(self):
		self.subscriber = None
		self.weights = {}
		self.leader = None

	def run(self):
		#every 2 seconds
		rate = rospy.Rate(0.5)
		now = rospy.get_rostime()
		while self.active and not rospy.is_shutdown():
			lastEpoch = now
			rate.sleep()
			now = rospy.get_rostime()

			self.connectedLock.acquire(shared=True)
			#get a snapshot of the connectedClients
			connectedSnapshot = self.getConnectedClients()

			#calculate who the leader is
			newLeader = self.calculateLeader(now, connectedSnapshot)
			self.connectedLock.release()

			if self.leader:
				rospy.loginfo("aggregator leader: %s - %s", self.leader.topic, self.weights)
			if not newLeader == self.leader and newLeader is not None:
				self.leader = newLeader
				self.swapForwarding()
				leaderMsgs = self.leader.msgsFromEpoch(now)
				if leaderMsgs:
					self.forwardingCallback(leaderMsgs[-1][1])


	def swapForwarding(self):
		if self.subscriber:
			self.subscriber.unregister()

		self.subscriber = rospy.Subscriber(self.leader.topic, Twist, self.forwardingCallback)

	def forwardingCallback(self, twist):
		self.publish(twist)

	def getWeight(self, topic):
		if topic not in self.weights:
			self.weights[topic] = 0.5
		return self.weights[topic]

	def calculateLeader(self, now, connectedSnapshot):
		historicalAgreementFactor = 0.94

		#get the crowd action vector
		crowdActionVector, clientActionVectors = self.crowdsEpochActionVectors(now, connectedSnapshot)
		if clientActionVectors is None:
			return None

		for topic in connectedSnapshot:
			client = connectedSnapshot[topic]

			vc = self.calculateVC(clientActionVectors.get(topic,None), crowdActionVector)

			#if the client has not voted this epoch, begin decaying their weight
			if not vc:
				vc = 0

			#update the weights
			self.weights[topic] = self.getWeight(topic)*historicalAgreementFactor + (1-historicalAgreementFactor)*vc

		#client with the largest weight is the leader
		largestWeight = 0
		largestClient = None
		for topic in self.weights:
			if self.weights[topic] > largestWeight:
				largestWeight = self.weights[topic]
				largestClient = connectedSnapshot[topic]
		return largestClient


	def calculateVC(self, clientAV, crowdAV):
		if crowdAV == None or clientAV == None:
			return None

		#calculate Dot Product
		vectorcosine = crowdAV['F']*clientAV['F'] + crowdAV['B']*clientAV['B'] + crowdAV['L']*clientAV['L'] + crowdAV['R']*clientAV['R']
		return vectorcosine


	def clientsEpochActionVectors(self, client, now):
		av = {}
		av['F'] = 0.0 #forward
		av['B'] = 0.0 #backward
		av['L'] = 0.0 #left
		av['R'] = 0.0 #right

		msgs = client.msgsFromEpoch(now)
		if not msgs:
			return None

		for msg in msgs:
			twist = msg[1]
			if(twist.linear.x > 0):
				av['F'] += 1.0
			elif(twist.linear.x < 0):
				av['B'] += 1.0

			elif(twist.angular.z > 0):
				av['L'] += 1.0
			elif(twist.angular.z < 0):
				av['R'] += 1.0

		vecMag = math.sqrt(av['F']*av['F'] + av['B']*av['B'] + av['L']*av['L'] + av['R']*av['R'])

		av['F'] = float(av['F']) / float(vecMag)
		av['B'] = float(av['B']) / float(vecMag)
		av['L'] = float(av['L']) / float(vecMag)
		av['R'] = float(av['R']) / float(vecMag)

		return av


	def crowdsEpochActionVectors(self, now, connectedSnapshot):
		#calculate crowd action vector
		cav = {}
		clientavs = {}
		cav['F'] = 0.0 #forward
		cav['B'] = 0.0 #backward
		cav['L'] = 0.0 #left
		cav['R'] = 0.0 #right

		for topic in connectedSnapshot:
			client = connectedSnapshot[topic]

			av = self.clientsEpochActionVectors(client, now)
			if av:
				clientavs[topic] = av

				cav['F'] += av['F']
				cav['B'] += av['B']
				cav['L'] += av['L']
				cav['R'] += av['R']

		vecMag = math.sqrt(cav['F']*cav['F'] + cav['B']*cav['B'] + cav['L']*cav['L'] + cav['R']*cav['R'])
		if vecMag == 0:
			return None, None
		
		cav['F'] = float(cav['F']) / float(vecMag)
		cav['B'] = float(cav['B']) / float(vecMag)
		cav['L'] = float(cav['L']) / float(vecMag)
		cav['R'] = float(cav['R']) / float(vecMag)
		return cav, clientavs
