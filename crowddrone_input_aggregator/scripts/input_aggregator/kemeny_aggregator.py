#!/usr/bin/env python
import rospy
from aggregator import Aggregator
from itertools import combinations, permutations
import numpy as np

class KemenyAggregator(Aggregator):
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

			#rank the votes and convert to a numpy array
			candidates = self.getCandidates()
			rankings = self.calculateRankings()

			#calculate Kemeny-Young rank aggregation
			min_dist, best_rank = rankaggr_brute(rankings)

			#publish the best ranked action
			if best_rank:
				rospy.loginfo("aggregator kemeny: %s"%min_dist)
				self.publish(candidates[best_rank.index(0)])

	def calculateRankings(self):
		rankings = []
		for topic in self.connectedClients:
			client = self.connectedClients[topic]
			if not client.isAlive:
				continue

			#make sure the clients latest action takes place in this epoch
			if not client.msgs:
				continue
			latestVote = client.msgs[-1]
			#we're not counting no action as a vote
			if self.twistIsEmpty(latestVote[1]):
				continue

			ranking = self.rankVote(latestVote[1])
			rankings.append(ranking)
		#convert to numpy array
		rankings = np.array(rankings)
		return rankings

def kendalltau_dist(rank_a, rank_b):
	#rank_a and rank_b are a list in order of candidates, that rank each candidate from 0 to n_candidates
	tau = 0
	n_candidates = len(rank_a)
	#for each combination of candidates
	for i, j in combinations(range(n_candidates), 2):
		#check that rank_a has candidate_i and candidate_j in the same order as rank_b, if not add 1 to the tau distance
		#How This Code Works:
			#np.sign(rank_a[i] - rank_a[j]) is essentially giving a -1 if candidate_i < candidate_j and +1 if candidate_i > candidate_j and 0 if the candidates have the same ranking
			#we then apply this same logic to rank_b
			#we check that this ordering is NOT equal by negating rank_b's order
			#in python adding Booleans, casts a False to 0 and True to 1, so if the ordering is NOT equal, this line will evaluate to True, and the tau distance is incremented
		tau += (np.sign(rank_a[i] - rank_a[j]) == -np.sign(rank_b[i] - rank_b[j]))
	return tau
def kendalltau_scipy(rank_a, rank_b):
	tau, pval = scipy.stats.kendalltau(rank_a, rank_b)
	return 1.0 - tau

def heuristic(chosenIndex, ranks):
	maxCost = 0
	for rank in ranks:
		cost = rank[chosenIndex]
		maxCost = max(maxCost,cost)
	return maxCost
def rankaggr_brute(ranks):
	min_dist = np.inf
	min_tau = np.inf
	min_cost = np.inf
	best_rank = None
	if ranks.size:
		n_voters, n_candidates = ranks.shape
		#loop through every possible ranking
		for candidate_rank in permutations(range(n_candidates)):
			#calculate the summed distance from this candidate ranking and all the rankings given
			dist = 0
			max_tau = 0
			for rank in ranks:
				tau = kendalltau_dist(candidate_rank, rank)
				if tau > max_tau:
					max_tau = tau
				dist = dist + tau
			#calculate a heuristic distance to decide between equal kendall tau distances
			max_cost = heuristic(candidate_rank.index(0), ranks)

			#if the distance is the smallest, its the most agreed upon ranking
			if dist < min_dist or (dist == min_dist and max_tau < min_tau) or (dist == min_dist and max_tau == min_tau and max_cost < min_cost):
				min_dist = dist
				min_tau = max_tau
				min_cost = max_cost
				best_rank = candidate_rank
	return min_dist, best_rank
