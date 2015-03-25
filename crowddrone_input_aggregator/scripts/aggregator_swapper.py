#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import String

if __name__ == "__main__":
	#initialize the ros node
	rospy.init_node("aggregator_swapper")

	pubChangeInput = rospy.Publisher("change_input_aggregator", String, queue_size=0)

	aggregatorList = ["mob","leader","realtime_weighted_vote","realtime_kemeny","realtime_kemeny_full"]
	doneAggregatorList = []

	while not rospy.is_shutdown():
		if not aggregatorList:
			aggregatorList = doneAggregatorList
			doneAggregatorList = []

		aggregator = random.choice(aggregatorList)
		aggregatorList.remove(aggregator)
		doneAggregatorList.append(aggregator)

		pubChangeInput.publish(String(aggregator))

		rospy.sleep(30)