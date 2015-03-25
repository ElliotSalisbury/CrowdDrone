#!/usr/bin/env python
import rospy
from aggregator import Aggregator
from geometry_msgs.msg import Twist

class VickreyKappaVoteAggregator(Aggregator):
	def initialize(self):
		self.weights = {}
		self.lastWeightCalc = rospy.get_rostime()

	def run(self):
		pass

	def notifyClientUpdated(self, topic):
		#calculate voter weight
		now = rospy.get_rostime()
		connectedSnapshot = self.getConnectedClients()

		voteMap, voteCount = self.getVoteMap(now, connectedSnapshot)

		#every second it will calculate new weights
		self.calculateWeightsForEpoch(now, voteMap, voteCount, connectedSnapshot)

		#get most voted action
		action, score = self.getMostVotedForAction(voteMap, voteCount)

		rospy.loginfo("aggregator vickrey_kappa: %s - %s", score, self.weights)
		self.publish(action)

	def getWeight(self, topic):
		if topic not in self.weights:
			self.weights[topic] = 0.5
		return self.weights[topic]

	def getVoteMapArray(self, voteMap, action):
		for vote in voteMap:
			if self.twistIsEqual(vote, action):
				return voteMap[vote]
		voteMap[action] = []
		return voteMap[action]

	def getVoteMap(self, now, connectedSnapshot):
		#count the number of votes each actions gets
		voteMap = {}
		voteCount = 0
		for topic in connectedSnapshot:
			client = connectedSnapshot[topic]
			msgs = client.msgsFromEpoch(now)

			#make sure the client is alive and that they have voted this epoch
			if not (client.isAlive and msgs):
				continue
			latestVote = msgs[-1][1]

			#add this client to the voteMap
			voters = self.getVoteMapArray(voteMap, latestVote)
			voters.append(topic)
			voteCount += 1

		return voteMap, voteCount

	def getMostVotedForAction(self, voteMap, voteCount):
		greatestAction = None
		greatestScore = 0
		for action in voteMap:
			score = 0
			for topic in voteMap[action]:
				score += self.getWeight(topic)
			if score > greatestScore:
				greatestAction = action
				greatestScore = score
		return greatestAction, greatestScore

	def calculateWeightsForEpoch(self, now, voteMap, voteCount, connectedSnapshot):
		historicalAgreementFactor = 0.0

		#check that now is greater than a second since last calculate
		if now >= self.lastWeightCalc + rospy.Duration(1) and voteCount > 2:
			#calculate agreement this round
			agreement = 0.0
			for vote in voteMap:
				thisVoteCount = len(self.getVoteMapArray(voteMap, vote))
				agreement += thisVoteCount*thisVoteCount
			agreement -= voteCount
			agreement /= voteCount*(voteCount-1.0)

			newWeights = {}
			for topic in connectedSnapshot:
				client = connectedSnapshot[topic]
				msgs = client.msgsFromEpoch(now)

				if msgs:
					latestVote = msgs[-1][1]
					#calculate agreement without this client
					agreementWithout = 0.0
					for vote in voteMap:
						thisVoteCount = len(self.getVoteMapArray(voteMap, vote))
						#if vote is the same as the vote from this user
						if self.twistIsEqual(vote,latestVote):
							thisVoteCount -= 1.0
						agreementWithout += thisVoteCount*thisVoteCount
					totalVoteCount = voteCount-1.0
					agreementWithout -= totalVoteCount
					agreementWithout /= totalVoteCount*(totalVoteCount-1.0)

					diff = agreement - agreementWithout

					agreementWeight = max(min(self.getWeight(topic)+diff,1),0) #add the different but keep within range
				else:
					continue #if they didnt vote, we can't say much about their weight
					#agreementWeight = self.getWeight(topic) * 0.95
					#agreementWeight = 0 #if they didnt vote start decaying their weight

				#calculate weight
				newWeights[topic] = (historicalAgreementFactor * self.getWeight(topic)) + ((1-historicalAgreementFactor)*agreementWeight)

			for topic in newWeights:
				self.weights[topic] = newWeights[topic]
			self.lastWeightCalc = now
