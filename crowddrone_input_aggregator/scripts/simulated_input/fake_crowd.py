import rospy
import sys
from crowddrone_input_aggregator.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from threading import Thread, Lock, Event

class FakeCrowd(object):
	def __init__(self, name="test"):
		#check that the get a new client topic exists
		try:
			rospy.wait_for_service('get_new_client_topic', 10)
		except rospy.ROSException:
			#timeout exceeded
			print("get_new_client_topic does not appear to be running")
			sys.exit(1)

		#get a new topic 
		try:
			get_new_topic = rospy.ServiceProxy('get_new_client_topic', GetNewClientTopic)

			req = GetNewClientTopicRequest()
			req.assignmentId="test"
			req.hitId="test"
			req.workerId=name
			req.turkSubmitTo="test"

			self.topic = get_new_topic(req).topic
		except rospy.ServiceException:
			#service call failed exceeded
			print("we failed to make the service call to get a new client topic")
			sys.exit(1)

		#now that we have a topic, lets create the publisher
		self.pub = rospy.Publisher(self.topic, Twist, queue_size=0)
		self.pubHeart = rospy.Publisher(self.topic+"/HB", Bool, queue_size=0)
		self.isRunning = False

		self.hbTimer = None
		self.loopThread = None
		self.lock = Lock()

	def start(self):
		with self.lock:
			if not self.loopThread:
				self.isRunning = True
				self.loopThread = Thread(target = self.run)
				self.loopThread.daemon = True
				self.loopThread.start()
				self.hbTimer = rospy.Timer(rospy.Duration(5), self.heartbeat)

	def stop(self):
		with self.lock:
			if self.loopThread:
				self.isRunning = False
				self.loopThread.join()
				self.loopThread = None
				self.hbTimer.shutdown()

	def checkIsRunning(self):
		with self.lock:
			return self.isRunning

	def run(self):
		pass

	def heartbeat(self, timerEvent):
		if not rospy.is_shutdown() and self.isRunning:
			self.pubHeart.publish(True)
