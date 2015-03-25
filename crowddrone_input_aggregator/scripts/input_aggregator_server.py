#!/usr/bin/env python

import sys
import importlib
from threading import Thread, Lock
from threading2 import SHLock
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from input_aggregator.aggregator import Aggregator
from crowddrone_input_aggregator.srv import *
import random

robotName = "ardrone"
connectedClients = {}
connectedLock = SHLock()
clientListeners = []
aggregatorInstance = None

def addClient(client):
	connectedClients[client.topic] = client

#incase clients are publishing to topics before this server runs
class TopicFinder(Thread):
	def __init__(self,namespace="/"):
		Thread.__init__(self)
		self.namespace = str(namespace)
	def run(self):
		#once a second, check if topics have been published without coming through the new client service
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			topics = rospy.get_published_topics(self.namespace)
			for topic in topics:
				if topic[0].endswith("HB"):
					continue #we dont want to connect to the heartbeat topics
				if not topic[0] in connectedClients:
					connectedLock.acquire(shared=False)
					addClient(Client(topic[0], "unknown", "unknown", "unknown", "unknown"))
					connectedLock.release()
			rate.sleep()

#handle a request to become a new client
def handle_new_client(req):
	connectedLock.acquire(shared=False)
	clientCount = len(connectedClients) + 1
	topic = "/input_aggregator/cmd_vel_%s"%clientCount
	while topic in connectedClients:
		clientCount = clientCount + 1
		topic = "/input_aggregator/cmd_vel_%s"%clientCount

	addClient(Client(topic, req.assignmentId, req.hitId, req.workerId, req.turkSubmitTo))
	connectedLock.release()
	return GetNewClientTopicResponse(topic)

def handle_connected_info(req):
	connectedLock.acquire(shared=True)
	count = 0
	hitIds = []
	workerIds = []
	assignmentIds = []

	for topic in connectedClients:
		client = connectedClients[topic]
		if client.isAlive:
			count += 1
			hitIds.append(client.hitId)
			workerIds.append(client.workerId)
			assignmentIds.append(client.assignmentId)
	connectedLock.release()
	return GetConnectedInfoResponse(count, hitIds, workerIds, assignmentIds)

class Client:
	def __init__(self,topic, assignmentId, hitId, workerId, turkSubmitTo):
		self.topic = topic
		self.assignmentId = assignmentId
		self.hitId = hitId
		self.workerId = workerId
		self.turkSubmitTo = turkSubmitTo
		self.isAlive = True
		self.lastHeartbeat = rospy.get_rostime()
		self.msgs = []
		self.msgLock = Lock()
		self.HOLDFOR = 5
		self.DURATION = 2
		self.fakeping = random.random() * 0
		rospy.loginfo("client connected: %s - %s %s %s %s", self.topic, self.assignmentId, self.hitId, self.workerId, self.turkSubmitTo)
		rospy.Subscriber(self.topic, Twist, self.msgReceived)
		rospy.Subscriber(self.topic+"/HB", Bool, self.heartbeatReceived)
		Thread(target=self.timeout).start()

	def msgReceived(self, msg):
		self.heartbeatReceived(msg)
		storedmsg = [rospy.get_rostime(), msg]
		with self.msgLock:
			self.msgs.append(storedmsg)
			self.clearOldMsgs(storedmsg[0])

		rospy.loginfo("client msgs: %s - %s", self.topic, storedmsg)

		Thread(target=self.notifyListeners).start()

	def clearOldMsgs(self, now):
		try:
			#clear anything older than 5 seconds
			old = now - rospy.Duration(self.HOLDFOR)
			self.msgs = [msg for msg in self.msgs if msg[0] > old]
		except TypeError: #catch negative times
			pass

	def msgsFromEpoch(self, now):
		try:
			old = now - rospy.Duration(self.DURATION)
		except TypeError: #catch negative times
			return []

		rmsgs = []
		if self.msgs:
			with self.msgLock:
				for index in reversed(xrange(len(self.msgs))):
					msg = self.msgs[index]
					if msg[0] <= now:
						if msg[0] > old:
							rmsgs.insert(0,msg)
						else:
							break
		return rmsgs

	def heartbeatReceived(self, msg):
		self.isAlive = True
		self.lastHeartbeat = rospy.get_rostime()
	def timeout(self):
		while not rospy.is_shutdown():
			if(self.isAlive and self.lastHeartbeat + rospy.Duration(10) < rospy.get_rostime()):
				self.isAlive = False
				
				rospy.loginfo("client timeout: %s",self.topic)
				
				self.notifyListeners()
			rospy.sleep(10.0)
	def notifyListeners(self):
		if self.fakeping > 0:
			rospy.sleep(self.fakeping)
		for listener in clientListeners:
			listener.notifyClientUpdated(self.topic)

def switchAggregator(moduleName):
	global aggregatorInstance
	if hasattr(moduleName, 'data'):
		moduleName=moduleName.data

	aggregatorClass = getModuleClass(moduleName)
	if aggregatorClass:
		if aggregatorInstance and aggregatorInstance.__class__ == aggregatorClass:
			return None #bail if the aggregator is the same

		if aggregatorInstance:
			clientListeners.remove(aggregatorInstance)
			aggregatorInstance.stop()
		aggregatorInstance = aggregatorClass(connectedClients, connectedLock, 'ai_decorator/'+robotName+'/cmd_vel')
		aggregatorInstance.start()
		clientListeners.append(aggregatorInstance)
		rospy.loginfo("aggregator swap: %s", moduleName)
		rospy.logwarn("aggregator swap: %s", moduleName)

def getModuleClass(moduleName):
	if not moduleName.endswith("_aggregator"):
		moduleName = moduleName + "_aggregator"
	moduleName = "input_aggregator." + moduleName
	module = None
	try:
		module = importlib.import_module(moduleName)
	except ImportError:
		rospy.logwarn("No module named: %s", moduleName)
		return None

	for name in dir(module):
		#we dont want to get the abstract baseclass
		if(name == "Aggregator"):
			continue
		#check if this name is a subclass of the Aggregator baseclass
		obj = getattr(module, name)
		try:
			if issubclass(obj, Aggregator):
				return obj
		except TypeError:  # If 'obj' is not a class
			pass
	return None

if __name__ == "__main__":
	#make sure we have the arguments for loading the aggregator class module
	argv = rospy.myargv(sys.argv)
	if not len(argv) == 3:
		print("Incorrect Usage: You must pass in the name of the aggregator module and the name of the robot to control\n%s [AggregatorModule] [RobotName]"%sys.argv[0])
		sys.exit(1)

	robotName = argv[2]
	#initialize the ros node, the topic finder and the new client topic service
	rospy.init_node("input_aggregator_server")
	TopicFinder("input_aggregator").start()
	rospy.Service('get_new_client_topic', GetNewClientTopic, handle_new_client)
	rospy.Service('get_connected_info', GetConnectedInfo, handle_connected_info)

	#initialize the aggregator class and start its thread
	switchAggregator(argv[1])
	rospy.Subscriber("change_input_aggregator", String, switchAggregator)

	rospy.spin()
