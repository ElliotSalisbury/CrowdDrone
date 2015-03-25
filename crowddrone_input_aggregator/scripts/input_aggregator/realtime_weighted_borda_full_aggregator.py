#!/usr/bin/env python
import rospy
from aggregator import Aggregator
import math
import numpy as np
from geometry_msgs.msg import Twist

class RealtimeWeightedBordaAggregator(Aggregator):
	def initialize(self):
		self.weights = {}
		self.lastWeightCalc = rospy.get_rostime()

	def run(self):
		pass

	def notifyClientUpdated(self, topic):
		#rank the votes and convert to a numpy array
		linearCandidates = self.getLinearCandidates()
		angularCandidates = self.getAngularCandidates()

		linearRankings, angularRankings = rankings = self.calculateRankings(rospy.get_rostime())

		#sum the rankings of each candidate
		linearBordaCount = np.sum(linearRankings, axis=0)
		angularBordaCount = np.sum(angularRankings, axis=0)

		#highest ranked candidate
		bestLinearCandidateIndex = np.argmax(linearBordaCount)
		bestAngularCandidateIndex = np.argmax(angularBordaCount)
		bestLinearCandidate = linearCandidates[bestLinearCandidateIndex]
		bestAngularCandidate = angularCandidates[bestAngularCandidateIndex]

		#publish the best ranked action
		rospy.loginfo("aggregator realtime_weighted_borda_full: %s - %s",(bestLinearCandidate,bestAngularCandidate),self.weights)

		action=Twist()
		if bestLinearCandidate:
			action.linear = bestLinearCandidate.linear
		if bestAngularCandidate:
			action.angular = bestAngularCandidate.angular
		self.publish(action)

	def calculateRankings(self, now):
		linearRankings = []
		angularRankings = []
		connectedSnapshot = self.getConnectedClients()
		self.connectedLock.acquire(shared=True)
		#update the weights in here while we have a lock
		self.calculateWeightsForEpoch(now, connectedSnapshot)

		for topic in connectedSnapshot:
			client = connectedSnapshot[topic]
			msgs = client.msgsFromEpoch(now)

			#make sure the client is alive and that they have voted this epoch
			if not (client.isAlive and msgs):
				continue

			latestVote = msgs[-1][1]
			#we're not counting no action as a vote
			if self.twistIsEmpty(latestVote):
				continue

			linearRanking = np.array(self.rankLinearVote(latestVote))
			angularRanking = np.array(self.rankAngularVote(latestVote))

			#convert the rankings from lowest number means more preferred to highest number means more preferred
			linearRanking = len(linearRanking) - linearRanking
			angularRanking = len(angularRanking) - angularRanking

			#multiply ranking by the users weight
			linearRanking = linearRanking * self.getWeight(topic)
			angularRanking = angularRanking * self.getWeight(topic)

			linearRankings.append(linearRanking)
			angularRankings.append(angularRanking)
		self.connectedLock.release()

		#convert to numpy array
		linearRankings = np.array(linearRankings)
		angularRankings = np.array(angularRankings)
		return linearRankings, angularRankings

	def getWeight(self, topic):
		if topic not in self.weights:
			self.weights[topic] = 0.5
		return self.weights[topic]

	def calculateWeightsForEpoch(self, now, connectedSnapshot):
		historicalAgreementFactor = 0.94

		#check that now is greater than a second since last calculate
		if now >= self.lastWeightCalc + rospy.Duration(1):
			#get the crowd action vector
			crowdActionVector, clientActionVectors = self.crowdsEpochActionVectors(now, connectedSnapshot)
			if not crowdActionVector or not clientActionVectors:
				return

			for topic in connectedSnapshot:
				client = connectedSnapshot[topic]

				vc = self.calculateVC(clientActionVectors.get(topic,None), crowdActionVector)

				#if the client has not voted this epoch, begin decaying their weight
				if not vc:
					vc = 0

				#update the weights
				self.weights[topic] = self.getWeight(topic)*historicalAgreementFactor + (1-historicalAgreementFactor)*vc
			self.lastWeightCalc = now


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
