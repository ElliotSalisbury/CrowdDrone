#!/usr/bin/env python
import math
from gazebo_model_stater import GazeboModelStater
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from path_from_rings import getPathFromRings

class DroneController():
	def __init__(self, robotName):
		self.gazeboModelStater = GazeboModelStater()
		self.path = []
		self.lookAhead = 1.5
		self.errorMultiplier = 1
		self.currentSegment = (-1,-1)
		self.robotName = robotName

	def addUpdateCallback(self, callback):
		self.gazeboModelStater.addUpdateCallback(callback)

	def setPath(self, path):
		self.path = getPathFromRings(self.gazeboModelStater, self.robotName, path)

	def getDroneState(self):
		return self.gazeboModelStater.get(self.robotName)

	def getClosestLineSegment(self, p):
		closestDist = 99999
		closestSegment = (-1,-1)
		closestProjected = [0,0]
		shortestJump = 999999
		for i in range(len(self.path)):
			j = self.addToIndex(i,1)

			a = self.path[i]
			b = self.path[j]

			jump = abs(i-self.currentSegment[0])
			jumpFromLastSegment = min(jump, abs(jump-len(self.path)))

			distance, projected = distToSegment2(a,b,p)
			if distance < closestDist:
				if closestDist-distance > 1 or jumpFromLastSegment < shortestJump:
					closestDist = distance
					closestSegment = (i,j)
					closestProjected = projected
					shortestJump = jumpFromLastSegment

		return closestSegment, closestDist, closestProjected

	def isForward(self, yaw, currentSegment):
		a = self.path[currentSegment[0]]
		b = self.path[currentSegment[1]]

		dir = [math.cos(yaw), math.sin(yaw)]

		dp = dot(dir,sub(b,a))
		if dp > 0:
			return True
		else:
			return False

	def getControlInfo(self):
		currentPose, currentTwist = self.gazeboModelStater.get(self.robotName)
		if not currentPose:
			return 0
		pos = [currentPose.position.x, currentPose.position.y]
		pitch, roll, yaw = euler_from_quaternion([currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w])

		#calculate where we are on the path, and where we should be
		currentSegment, posErr, projected = self.getClosestLineSegment(pos)
		self.currentSegment = currentSegment
		#are we facing forward
		isForward = self.isForward(yaw, currentSegment)

		#calculate where we should drive too
		i = currentSegment[0]
		j = currentSegment[1]
		a = self.path[i]
		b = self.path[j]

		if isForward:
			dirMult = 1
		else:
			dirMult = -1

		remainingLookAhead = self.lookAhead
		while remainingLookAhead > 0.01:
			follow = add(projected,scale(normalize(sub(b,a)),dirMult*remainingLookAhead))
			remainingLookAhead, projected = distToSegment2(a,b,follow)

			i = self.addToIndex(i, dirMult)
			j = self.addToIndex(j, dirMult)
			a = self.path[i]
			b = self.path[j]

		positionError = sub(follow,pos)
		targetYaw = math.atan2(positionError[1], positionError[0])
		angleError = targetYaw - yaw

		while angleError > math.pi:
			angleError -= 2*math.pi
		while angleError < -math.pi:
			angleError += 2*math.pi
		angleError *= self.errorMultiplier
		return angleError, posErr, currentSegment, isForward

	def addToIndex(self, index, add):
		newi = index + add
		if newi < 0:
			newi += len(self.path)
		else:
			newi = newi % len(self.path)
		return newi

def sqr(x):
	return x*x
def dist2(a, b):
	return sqr(b[0]-a[0]) + sqr(b[1]-a[1])
def dist(a, b):
	return math.sqrt(dist2(a,b))

def add(a, b):
	return [a[0]+b[0], a[1]+b[1]]
def sub(a, b):
	return [a[0]-b[0], a[1]-b[1]]
def neg(a):
	return [-a[0], -a[1]]
def scale(a, s):
	return[a[0]*s, a[1]*s]

def distToSegment2(a,b, p):
	l2 = dist2(a,b)
	#if line segment has no length, then a,b are same point, calc distance of p from a
	if l2 == 0:
		return dist2(a,p), a

	#consider line segment as a + t(b-a)
	#project p onto that line
	#t = [(p-a) . (b-a)] / |b-a|^2
	t = dot(sub(p,a),sub(b,a)) / l2

	if t < 0:
		return dist2(a,p), a
	elif t > 1:
		return dist2(b,p), b
	else:
		projected = add(a,scale(sub(b,a),t))
		return dist2(projected, p), projected

def normalize(a):
	length = math.sqrt(a[0]*a[0]+a[1]*a[1])
	return [a[0]/length, a[1]/length]


def dot(a, b):
	return a[0]*b[0] + a[1]*b[1]

