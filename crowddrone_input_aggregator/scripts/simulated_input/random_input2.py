#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from fake_crowd import FakeCrowd
import random

class RandomInput(FakeCrowd):
	def run(self):
		while not rospy.is_shutdown() and self.isRunning:
			twist = Twist()
			type = random.randint(1,2)
			if type == 1:
				twist.angular.z=-1
			elif type == 2:
				twist.linear.x=-1

			self.pub.publish(twist)

			delay = (random.random() * 1.8) + 0.2
			rospy.sleep(delay)

if __name__ == "__main__":
	rospy.init_node("random_input", anonymous=True)
	name = rospy.get_name()
	random_input = RandomInput(name)
	random_input.start()

	rospy.spin()
