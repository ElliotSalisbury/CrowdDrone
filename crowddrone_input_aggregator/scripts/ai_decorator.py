#!/usr/bin/env python

import math
import rospy
import roslaunch
from tf import transformations
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from simulated_input.gazebo_model_stater import GazeboModelStater
import sys
import subprocess, signal
import os
import random

gazeboModelStater = GazeboModelStater()
robotName = "ardrone"
desiredTwist = Twist()
bounceBack = [0,0]
currentPose = None
desiredZ = 1.3
posError = 0
cmdvelpub = None
takeoffpub = None
takeoffcount = 20
collisionRects = []
eventRects = []
spawnedAlready = {}

def msgReceived(msg):
	global desiredTwist
	desiredTwist = msg
	publishMsg()

def updateError():
	global currentPose, bounceBack
	currentPose, currentTwist = gazeboModelStater.get(robotName)
	if currentPose:
		global posError
		posError = desiredZ - currentPose.position.z

		#loop through the event rects
		for rect in eventRects:
			minx = rect[0][0]
			miny = rect[0][1]
			maxx = rect[1][0]
			maxy = rect[1][1]
			callback = rect[2]

			#check if collision
			if currentPose.position.x >= minx and currentPose.position.x <= maxx and currentPose.position.y >= miny and currentPose.position.y <= maxy:
				callback(rect, currentPose.position)

		#loop through collision rects and bounce back
		bounceBack[0] = 0
		bounceBack[1] = 0
		for rect in collisionRects:
			minx = rect[0][0]
			miny = rect[0][1]
			maxx = rect[1][0]
			maxy = rect[1][1]

			#check if collision
			if currentPose.position.x >= minx and currentPose.position.x <= maxx and currentPose.position.y >= miny and currentPose.position.y <= maxy:
				#closest x dir
				halfx = minx + ((maxx - minx)/2.0)
				halfy = miny + ((maxy - miny)/2.0)

				bounceBack[0] = currentPose.position.x - halfx
				bounceBack[1] = currentPose.position.y - halfy

				if currentPose.position.x < halfx:
					xWall = minx
					xBounce = 1
				else:
					xWall = maxx
					xBounce = -1

				if currentPose.position.y < halfy:
					yWall = miny
					yBounce = -1
				else:
					yWall = maxy
					yBounce = 1

				xDist = abs(currentPose.position.x - xWall)
				yDist = abs(currentPose.position.y - yWall)
				if xDist < yDist:
					bounceBack[0] = xBounce
					bounceBack[1] = 0
				else:
					bounceBack[0] = 0
					bounceBack[1] = yBounce



			#if currentPose.position.x > 9:
#				bounceBack[0] = 1
			#elif currentPose.position.x < -9:
#				bounceBack[0] = -1
			#else:
#				bounceBack[0] = 0
#
		#if currentPose.position.y > 4.9:
#			bounceBack[1] = -1
		#elif currentPose.position.y < -4.9:
#			bounceBack[1] = 1
#		else:
			#bounceBack[1] = 0

def publishMsg():
	global takeoffcount
	updateError()
	pubTwist = cloneTwist(desiredTwist)
	pubTwist.linear.z = posError * 4

	if currentPose:
		if abs(bounceBack[0]) == 1 or abs(bounceBack[1]) == 1:
			quat = [currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w]
			euler = transformations.euler_from_quaternion(quat)
			cs = math.cos(euler[2])
			sn = math.sin(euler[2])
			px = bounceBack[0] * cs - bounceBack[1] * sn
			py = bounceBack[0] * sn + bounceBack[1] * cs

			pubTwist.linear.x = -px
			pubTwist.linear.y = py

	cmdvelpub.publish(pubTwist)

	if currentPose and currentPose.position.z < 0.1 and takeoffcount >= 20:
		takeoffpub.publish(Empty())
		takeoffcount = 0
	takeoffcount = takeoffcount + 1

def cloneTwist(twist):
	newTwist = Twist()
	newTwist.linear.x = twist.linear.x
	newTwist.linear.y = twist.linear.y
	newTwist.linear.z = twist.linear.z
	newTwist.angular.x = twist.angular.x
	newTwist.angular.y = twist.angular.y
	newTwist.angular.z = twist.angular.z
	return newTwist

def finishCallback(rect, pos):
	#rospy.signal_shutdown("Drone At End Point")
	rospy.loginfo("Drone At End Point")
	p = subprocess.Popen(['pkill', 'roslaunch'], stdout=subprocess.PIPE)
	out, err = p.communicate()
	#for line in out.splitlines():
	#	if 'roslaunch' in line:
	#		pid = int(line.split(None, 1)[0])
	#		os.kill(pid, signal.SIGKILL)

def spawnInfrontCallback(rect, pos):
	global spawnedAlready

	#xSpawn is maximum x of rectangle
	xSpawn = rect[1][0]

	if xSpawn not in spawnedAlready:
		spawnedAlready[xSpawn] = True

		#get drones position so we spawn near it
		haveToSpawn = []
		droneY = pos.y
		if droneY >= 3:
			haveToSpawn.append(9)
			haveToSpawn.append(3)
		elif droneY < 3 and droneY >= -3:
			haveToSpawn.append(3)
			haveToSpawn.append(-3)
		else:
			haveToSpawn.append(-3)
			haveToSpawn.append(-9)

		randomSpawns = [-9,-3,3,9]
		for spawn in haveToSpawn:
			randomSpawns.remove(spawn)
		haveToSpawn.append(random.choice(randomSpawns))

		rospy.loginfo("Spawn Blockades %s - %s"%(xSpawn, haveToSpawn))

		#spawn the blocks
		for ySpawn in haveToSpawn:
			spawnBlock(xSpawn,ySpawn, False)

def spawnBlock(x,y, rotated=False):
	if rotated:
		colRect = [[x-3.8, y-0.8], [x+3.8, y+0.8]]
		collisionRects.append(colRect)
		yaw = 1.57079633
	else:
		colRect = [[x-0.8, y-3.8], [x+0.8, y+3.8]]
		collisionRects.append(colRect)
		yaw = 0
	ident = "%s%s"%(x,y+10)
	p = subprocess.Popen(['rosrun gazebo_ros spawn_model -sdf -database blockade -model blockade%s -x %s -y %s -Y %s' % (ident,x,y,yaw)], stdout=subprocess.PIPE, shell=True)
	out, err = p.communicate()

if __name__ == "__main__":
	argv = rospy.myargv(sys.argv)
	if len(argv) >= 2:
		robotName = sys.argv[1]

	#get the collision rects from the command line
	if len(argv) >= 3:
		collisionRects = eval(sys.argv[2])

	mode = 1
	if len(argv) >= 4:
		mode = int(sys.argv[3])

	#generate the finish event rect
	if mode == 2:
		eventRects.append([[4.5,-1.5],[7.5,1.5],finishCallback])
	elif mode >= 3:
		eventRects.append([[38.5,-1.5],[41.5,1.5],finishCallback])

		eventRects.append([[6,-14],[9,14],spawnInfrontCallback])
		eventRects.append([[13,-14],[16,14],spawnInfrontCallback])
		eventRects.append([[20,-14],[23,14],spawnInfrontCallback])
		eventRects.append([[27,-14],[30,14],spawnInfrontCallback])
		eventRects.append([[34,-14],[37,14],spawnInfrontCallback])


	#initialize the ros node, and the topic for decorating twist msgs
	rospy.init_node("ai_decorator")
	rospy.Subscriber('ai_decorator/'+robotName+'/cmd_vel', Twist, msgReceived)

	cmdvelpub = rospy.Publisher('/'+robotName+'/cmd_vel', Twist, queue_size=0)
	takeoffpub = rospy.Publisher('/'+robotName+'/takeoff', Empty, queue_size=0)

	#start the control loop
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		publishMsg()
		rate.sleep()