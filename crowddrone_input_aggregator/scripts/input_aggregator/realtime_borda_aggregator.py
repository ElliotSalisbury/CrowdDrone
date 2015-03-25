#!/usr/bin/env python
import rospy
from aggregator import Aggregator
from itertools import combinations, permutations
import numpy as np

class RealtimeBordaAggregator(Aggregator):
	def initialize(self):
		self.weights = {}

	def run(self):
		pass

	def notifyClientUpdated(self, topic):
		#rank the votes and convert to a numpy array
		candidates = self.getCandidates()
		rankings = self.calculateRankings(rospy.get_rostime())

		#convert the rankings from lowest number means more preferred to highest number means more preferred
		rankings = len(candidates) - rankings

		#sum the rankings of each candidate
		bordaCount = np.sum(rankings, axis=0)

		#highest ranked candidate
		bestCandidateIndex = np.argmax(bordaCount)
		bestCandidate = candidates[bestCandidateIndex]

		#publish the best ranked action
		rospy.loginfo("aggregator realtime_borda: %s"%bordaCount)
		self.publish(bestCandidate)

	def calculateRankings(self, now):
		rankings = []
		connectedSnapshot = self.getConnectedClients()
		self.connectedLock.acquire(shared=True)
		for topic in connectedSnapshot:
			client = connectedSnapshot[topic]
			msgs = client.msgsFromEpoch(now)

			#make sure the client is alive and that they have voted this epoch
			if not (client.isAlive and msgs):
				continue

			latestVote = msgs[-1][1]
			ranking = self.rankVote(latestVote)
			rankings.append(ranking)
		self.connectedLock.release()
		#convert to numpy array
		rankings = np.array(rankings)
		return rankings