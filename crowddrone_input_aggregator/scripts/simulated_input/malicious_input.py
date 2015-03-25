#!/usr/bin/env python
import rospy
import sys
import math
from crowddrone_input_aggregator.srv import *
from geometry_msgs.msg import Twist
from gazebo_model_stater import GazeboModelStater
from drone_controller import DroneController
from fake_crowd import FakeCrowd

robotName = "ardrone"

class MaliciousInput(FakeCrowd):
	def run(self):
		#create the drone controller
		droneController = DroneController(robotName)
		path = [1,2,3,4,5,6,7,1,8,9,10,11,12,13]
		droneController.setPath(path)

		while not rospy.is_shutdown():
			twist = Twist()

			angle = droneController.getAngleError()
			#if math.fabs(angle) > 0.1:
			twist.angular.z=angle
			#else:
			twist.linear.x=1
	
			self.pub.publish(twist)
			rospy.sleep(1)

if __name__ == "__main__":
	global robotName
	argv = rospy.myargv(sys.argv)
	robotName = "ardrone"
	if len(argv) == 2:
		robotName = sys.argv[1]

	#initialize the ros node
	rospy.init_node("malicious_input", anonymous=True)
	malicious = MaliciousInput()

	#start the fake crowd
	malicious.start()

	rospy.spin()
