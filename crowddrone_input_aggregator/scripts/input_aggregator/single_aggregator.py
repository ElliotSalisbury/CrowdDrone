#!/usr/bin/env python
import rospy
from aggregator import Aggregator

class SingleAggregator(Aggregator):
	def initialize(self):
		self.singleTopic = None

	def run(self):
		pass

	def notifyClientUpdated(self, topic):
		if not self.singleTopic:
			self.singleTopic = topic

		if topic == self.singleTopic:
			client = self.connectedClients[self.singleTopic]
			msgs = client.msgsFromEpoch(rospy.get_rostime())
			if msgs:
				action = msgs[-1][1]
				rospy.loginfo("aggregator single: forwarding")
				self.publish(action)
