# ## ##################################################################
# controlnode.py
#
# ROS node for control GUI in python
#
# @author: Mauricio Matamoros
#
# ## ##################################################################
import re
import rospy
import threading
from std_msgs.msg import String as rosstr

class ControlNode(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self._rxWatch = re.compile(r'watching:(\d+)')
		self._initialized = False
		self._gui = None
	#end def


	# ## ##############################################################
	#
	# Properties
	#
	# ## ##############################################################
	@property
	def gui(self):
		return self._gui
	#end def

	@gui.setter
	def gui(self, value):
		self._gui = value
	#end def


	# ## ##############################################################
	#
	# Public methods
	#
	# ## ##############################################################
	def init(self):
		if self._initialized:
			return
		self._initialized = True

		rospy.init_node('CLIPSpyControlGUI')
		self._rate = rospy.Rate(10)
		self._pub = rospy.Publisher('clips_in', rosstr, queue_size=5)
		self._subcls = rospy.Subscriber('clips_status', rosstr, self._clstatSubsCallback, queue_size=1)
	#end def


	def publish(self, s:str):
		self._pub.publish(s)
	#end def

	def run(self):
		rospy.spin()
	#end def


	# ## ##############################################################
	#
	# Private methods
	#
	# ## ##############################################################

	def _clstatSubsCallback(self, msg):
		match = self._rxWatch.search(msg.data)
		if not match:
			return
		flags = int(match.group(1))
		self._gui.setWatchFlags(flags)
	#end def

#end class
