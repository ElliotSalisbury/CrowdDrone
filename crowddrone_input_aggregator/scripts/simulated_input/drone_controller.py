#!/usr/bin/env python
import math
from gazebo_model_stater import GazeboModelStater
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from path_from_rings import getPathFromRings

class DroneController():
	def __init__(self, robotName):
		self.gazeboModelStater = GazeboModelStater()
		self.path = []
		self.cPathI = 0
		self.lookAhead = 1.5
		self.errorMultiplier = 1
		self.robotName = robotName

	def addUpdateCallback(self, callback):
		self.gazeboModelStater.addUpdateCallback(callback)

	def setPath(self, path):
		self.path = getPathFromRings(self.gazeboModelStater, self.robotName, path)

	def getDroneState(self):
		return self.gazeboModelStater.get(self.robotName)

	def getPathIndex(self):
		return self.cPathI

	def getPositionError(self):
		currentPose, currentTwist = self.gazeboModelStater.get(self.robotName)
		if not currentPose:
			return 0

		robotCoord = [currentPose.position.x, currentPose.position.y]

		#calculate path index
		while True:
			indexback = self.subIndex(self.cPathI)
			index = self.cPathI
			indexfront = self.addIndex(self.cPathI)

			if isPassed(robotCoord, self.path[index], self.path[indexback], self.path[indexfront]):
				self.cPathI = self.addIndex(self.cPathI)
			else:
				break
	
		cPathI1 = self.addIndex(self.cPathI)
	
		dirVect = normalize(add(self.path[cPathI1], neg(self.path[self.cPathI])))
		projection = getProjection(robotCoord, self.path[self.cPathI], dirVect)
		posError = math.sqrt(distance2(robotCoord, projection))
		return posError

	def getAngleError(self):
		currentPose, currentTwist = self.gazeboModelStater.get(self.robotName)
		if not currentPose:
			return 0

		robotCoord = [currentPose.position.x, currentPose.position.y]
		quaternionArray = [currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w]
		roll, pitch, robotYaw = euler_from_quaternion(quaternionArray)

		#calculate path index
		while True:
			indexback = self.subIndex(self.cPathI)
			index = self.cPathI
			indexfront = self.addIndex(self.cPathI)

			if isPassed(robotCoord, self.path[index], self.path[indexback], self.path[indexfront]):
				self.cPathI = self.addIndex(self.cPathI)
			else:
				break
	
		cPathI1 = self.addIndex(self.cPathI)
	
		dirVect = normalize(add(self.path[cPathI1], neg(self.path[self.cPathI])))
		projection = getProjection(robotCoord, self.path[self.cPathI], dirVect)
		followPoint = add(projection, scale(dirVect, self.lookAhead))

		#ensure follow point remains on path, could be adding lookAhead directs it away from next path coords
		followI = self.cPathI
		while True:
			index1 = self.addIndex(followI)

			segLen2 = distance2(self.path[followI], self.path[index1])
			followLen2 = distance2(self.path[followI], followPoint)
			if followLen2 > segLen2:
				#should place it to the next segment
				index2 = self.addIndex(index1)
				extraLen = math.sqrt(followLen2) - math.sqrt(segLen2)
				dirVect = normalize(add(self.path[index2], neg(self.path[index1])))
				followPoint = add(self.path[index1], scale(dirVect, extraLen))
				followI = index1
			else:
				break

		positionError = add(neg(robotCoord), followPoint)
		targetYaw = math.atan2(positionError[1], positionError[0])
		angleError = targetYaw - robotYaw

		while angleError > math.pi:
			angleError -= 2*math.pi
		while angleError < -math.pi:
			angleError += 2*math.pi
		return angleError * self.errorMultiplier

	def subIndex(self, index):
		newi = index - 1
		if newi < 0:
			newi = newi + len(self.path)
		return newi

	def addIndex(self, index):
		newi = (index + 1) % len(self.path)
		return newi

def distance2(a, b):
	return (b[0]-a[0])*(b[0]-a[0]) + (b[1]-a[1])*(b[1]-a[1])

def add(a, b):
	return [a[0]+b[0], a[1]+b[1]]
    
def neg(a):
	return [-a[0], -a[1]]

def normalize(a):
	length = math.sqrt(a[0]*a[0]+a[1]*a[1])
	return [a[0]/length, a[1]/length]

def scale(a, s):
	return[a[0]*s, a[1]*s]

def dot(a, b):
	return a[0]*b[0] + a[1]*b[1]

def getProjection(point, linea, dirVect):
	relativePoint = [point[0]-linea[0], point[1]-linea[1]]
	distance = dot(relativePoint, dirVect)	
	return add(linea, scale(dirVect, distance))

def isPassed(currentPos, point, pointBack, pointFront):
	vectorBack = normalize(add(pointBack, neg(point)))
	vectorFront = normalize(add(pointFront, neg(point)))
	distBack = distance2(add(point, vectorBack), currentPos)
	distFront = distance2(add(point, vectorFront), currentPos)
	if(distBack <= distFront):
		return False
	else:
		return True
