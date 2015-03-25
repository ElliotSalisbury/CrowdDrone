#!/usr/bin/env python
import rospy
from aggregator import Aggregator
from geometry_msgs.msg import Twist

class RealtimelyWeightedVoteAggregator(Aggregator):
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

		rospy.loginfo("aggregator realtimely_weighted_vote: %s - %s", score, self.weights)
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
		self.connectedLock.acquire(shared=True)
		for topic in connectedSnapshot:
			client = connectedSnapshot[topic]
			msgs = client.msgsFromEpoch(now)

			#make sure the client is alive and that they have voted this epoch
			if not (client.isAlive and msgs):
				continue
			latestVote = msgs[-1][1]

			#calculate the timing weight
			timeWeight = ((rospy.Duration(client.DURATION) - (now - msgs[-1][0])).to_sec() / rospy.Duration(client.DURATION).to_sec())**1

			#add this client to the voteMap
			voters = self.getVoteMapArray(voteMap, latestVote)
			voters.append((topic, timeWeight))
			voteCount += 1
		self.connectedLock.release()

		return voteMap, voteCount

	def getMostVotedForAction(self, voteMap, voteCount):
		greatestAction = None
		greatestScore = 0
		for action in voteMap:
			score = 0
			for topic in voteMap[action]:
				score += self.getWeight(topic[0]) * topic[1]
			if score > greatestScore:
				greatestAction = action
				greatestScore = score
		return greatestAction, greatestScore

	def calculateWeightsForEpoch(self, now, voteMap, voteCount, connectedSnapshot):
		historicalAgreementFactor = 0.94

		#check that now is greater than a second since last calculate
		if now >= self.lastWeightCalc + rospy.Duration(1):
			# #calculate total weight
			totalWeight = 0
			agreementWeights = {}
			for vote in voteMap:
				agreementWeight = 0
				for voter in self.getVoteMapArray(voteMap, vote):
					totalWeight += self.getWeight(voter[0])
					agreementWeight += self.getWeight(voter[0])
				agreementWeights[vote] = agreementWeight
			newWeights = {}
			for vote in voteMap:
				for voter in self.getVoteMapArray(voteMap, vote):
					topic = voter[0]

					#normalize the summed weight
					agreementWeight = float(agreementWeights[vote]) / float(totalWeight)

					#calculate weight
					newWeights[topic] = (historicalAgreementFactor * self.getWeight(topic)) + ((1-historicalAgreementFactor)*agreementWeight)

			for topic in newWeights:
				self.weights[topic] = newWeights[topic]
			self.lastWeightCalc = now
