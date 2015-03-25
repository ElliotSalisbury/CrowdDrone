#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates

class GazeboModelStater():
	def __init__(self):
		rospy.Subscriber("gazebo/model_states", ModelStates, self.modelStateCallback)
		self.model_states = None
		self.listeners = []

	def modelStateCallback(self, model_states):
		self.model_states = model_states
		self.notifyListeners()

	def get(self, model):
		if(self.model_states):
			try:
				index = self.model_states.name.index(str(model))
				return self.model_states.pose[index], self.model_states.twist[index]
			except ValueError:
				pass
		return None, None

	def hasState(self):
		return self.model_states is not None

	def addUpdateCallback(self, callback):
		self.listeners.append(callback)

	def notifyListeners(self):
		for callback in self.listeners:
			callback()
			
