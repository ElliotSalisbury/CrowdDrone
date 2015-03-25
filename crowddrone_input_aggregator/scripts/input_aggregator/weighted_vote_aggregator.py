#!/usr/bin/env python
import rospy
from aggregator import Aggregator
from geometry_msgs.msg import Twist

class WeightedVoteAggregator(Aggregator):
	def initialize(self):
		self.weights = {}

	def run(self):
		#every second
		rate = rospy.Rate(1)
		newEpoch = rospy.get_rostime()
		while self.active and not rospy.is_shutdown():
			lastEpoch = newEpoch
			rate.sleep()
			newEpoch = rospy.get_rostime()

			#calculate voter weight
			self.calculateWeightsForEpoch()
			#get most voted action
			action = self.getMostVotedForAction()
			self.publish(action)

	def getMostVotedForAction(self):
		#count the number of votes each actions gets
		voteMap = {}
		for topic in self.connectedClients:
			client = self.connectedClients[topic]
			if not client.isAlive:
				continue
			
			#make sure the clients latest action takes place in this epoch
			if not client.msgs:
				continue
			latestVote = client.msgs[-1]
			
			#add this client to the voteMap
			if latestVote[1] not in voteMap:
				voteMap[latestVote[1]] = []
			voteMap[latestVote[1]].append(topic)

		greatestAction = None
		greatestScore = 0
		for action in voteMap:
			score = 0
			for topic in voteMap[action]:
				score = score+self.weights[topic]
			if score > greatestScore:
				greatestAction = action
				greatestScore = score
		return greatestAction
		

	def calculateWeightsForEpoch(self):
		historicalAgreementFactor = 0.8
		numClients = len(self.connectedClients)

		#count the number of votes each actions gets
		voteMap = {}
		for topic in self.connectedClients:
			client = self.connectedClients[topic]
			if not client.isAlive:
				continue
			
			#make sure the clients latest action takes place in this epoch
			if not client.msgs:
				continue
			latestVote = client.msgs[-1]
			
			#add this client to the voteMap
			if latestVote[1] not in voteMap:
				voteMap[latestVote[1]] = []
			voteMap[latestVote[1]].append(topic)

		for topic in self.connectedClients:
			client = self.connectedClients[topic]
			if not client.isAlive:
				continue
			#check if this client already has a weight
			if topic not in self.weights:
				self.weights[topic] = 1

			#make sure the clients latest action takes place in this epoch
			if not client.msgs:
				continue
			latestVote = client.msgs[-1]

			#sum the weights from other like minded voters
			agreementWeight = 0
			for topic2 in voteMap[latestVote[1]]:
				#check if other client already has a weight
				if topic2 not in self.weights:
					self.weights[topic2] = 1
				#sum the weight
				agreementWeight = agreementWeight + self.weights[topic2]
			#normalize the summed weight
			agreementWeight = agreementWeight/numClients
			
			#calculate weight
			self.weights[topic] = (historicalAgreementFactor * self.weights[topic]) + ((1-historicalAgreementFactor)*agreementWeight)
