#!/usr/bin/env python
import rospy
from aggregator import Aggregator

class MobAggregator(Aggregator):
	def initialize(self):
		pass

	def run(self):
		pass

	def notifyClientUpdated(self, topic):
		client = self.connectedClients[topic]
		msgs = client.msgsFromEpoch(rospy.get_rostime())
		if msgs:
			action = msgs[-1][1]
			rospy.loginfo("aggregator mob: forwarding")
			self.publish(action)
