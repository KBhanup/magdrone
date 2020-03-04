#!/catkin_ws/src/magdrone/launch

import rospy as rp
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers


class ar_listener:

	def __init__(self):
		rp.init_node("ar_marker_listener")

		#setup subscribers

		self.marker_sub = rp.Subscriber("/ar_pose_marker", AlvarMarkers, self.markerCB)

		rp.spin()

	def __del__(self):
		self.markerFile.close()

	def markerCB(self, msg):
		if (len(msg.markers) > 0):
			for i in range(len(msg.markers)):
				if msg.markers[i].id == 7:
					print "X position: ", msg.markers[i].pose.pose.position.x
					print "Y position: ", msg.markers[i].pose.pose.position.y
					print "Z position: ", msg.markers[i].pose.pose.position.z

#start node
saveAR = ar_listener()
