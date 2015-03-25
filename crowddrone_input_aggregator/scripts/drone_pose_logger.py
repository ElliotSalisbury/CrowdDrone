#!/usr/bin/env python

import rospy
import std_msgs
from simulated_input.drone_controller3 import DroneController
import sys

droneController = None
useRings = False

def callback():
	currentPose, currentTwist = droneController.getDroneState()
	angErr, posErr, pathSeg, isForward = droneController.getControlInfo()
	rospy.loginfo("pose: {\n%s\n}\ntwist: {\n%s\n}\ncontrol: {\npos: %s\nang: %s\nseg: %s\nforward: %s\n}", currentPose, currentTwist, posErr, angErr, pathSeg, isForward)

	if useRings:
		for ringid in path:
			topicname = "/ring%s/light" % ringid
			pub = rospy.Publisher(topicname, std_msgs.msg.Bool, queue_size=10)
			if ringid == pathSeg[1]:
				#turn light on
				pub.publish(std_msgs.msg.Bool(True))
			else:
				#turn light off
				pub.publish(std_msgs.msg.Bool(False))

if __name__ == "__main__":
	#initialize the ros node and DroneController, then register a callback
	rospy.init_node("drone_pose_logger")

	argv = rospy.myargv(sys.argv)
	robotName = "ardrone"
	if len(argv) >= 2:
		robotName = sys.argv[1]
	if len(argv) >= 3:
		mode = int(sys.argv[2])

	if mode==1:
		useRings = True
	

	droneController = DroneController(robotName)
	if useRings:
		path = [1,2,3,4,5,6,7,1,8,9,10,11,12,13]
		droneController.setPath(path)

	while not rospy.is_shutdown():
		callback()
		rospy.sleep(0.2)
	#droneController.addUpdateCallback(callback)
