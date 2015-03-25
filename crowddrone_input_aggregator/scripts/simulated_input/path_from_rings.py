import rospy

def getPathFromRings(gazeboModelStater, robotName, ringArray):
	path = []
	pose, twist = gazeboModelStater.get(robotName)
	while not pose:
		rospy.sleep(0.5)
		pose, twist = gazeboModelStater.get(robotName)

	for ringNum in ringArray:
		modelName = "ring" + str(ringNum)
		pose, twist = gazeboModelStater.get(modelName)
		path.append([pose.position.x, pose.position.y])

	return path
