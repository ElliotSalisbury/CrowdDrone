#!/usr/bin/env python
import rospy
import sys
import math
from crowddrone_input_aggregator.srv import *
from geometry_msgs.msg import Twist
from drone_controller import DroneController
from fake_crowd import FakeCrowd
import random

robotName = "ardrone"

class OptimalInput(FakeCrowd):
	def run(self):
		#create the drone controller
		droneController = DroneController(robotName)
		path = [1,2,3,4,5,6,7,1,8,9,10,11,12,13]
		droneController.setPath(path)

		#randomize how much this guy cares about angular error
		angleMult = (random.random() * 2) + 1.0

		while not rospy.is_shutdown() and self.isRunning:
			twist = Twist()

			angle = droneController.getAngleError()
			angle *= angleMult
			if math.fabs(angle) > 0.8:
				if angle > 0:
					twist.angular.z=1
				elif angle < 0:
					twist.angular.z=-1
			else:
				twist.linear.x=1
			self.pub.publish(twist)

			delay = (random.random() * 1.8) + 0.2
			rospy.sleep(delay)

if __name__ == "__main__":
	argv = rospy.myargv(sys.argv)
	robotName = "ardrone"
	if len(argv) == 2:
		robotName = sys.argv[1]

	#initialize the ros node
	rospy.init_node("optimal_input", anonymous=True)
	name = rospy.get_name()
	optimal = OptimalInput(name)

	#start the fake crowd
	optimal.start()

	rospy.spin()