#!/usr/bin/python

import rospy as rp
import numpy as np

import sys
import csv

from geometry_msgs.msg import PoseStamped

class ar_listener:
    '''
        Listen to the marker pose topic and the mocap pose topics and save all data in csv files
    '''

    def __init__(self, markerFile):
        rp.init_node("ar_marker_listener")

        # Open files
        self.markerFile = open(markerFile, 'w')

        # Write CSV headers
        fieldNames = ['Time', 'X', 'Y', 'Z', 'qX', 'qY', 'qZ', 'qW']
        
        self.markerOut = csv.DictWriter(self.markerFile, fieldnames=fieldNames)

        self.markerOut.writeheader()

        # Setup subscribers
        self.marker_sub = rp.Subscriber("/aruco_single/pose", PoseStamped, self.markerCB)

        rp.spin()

    def __del__(self):
        self.markerFile.close()

    def markerCB(self, msg):
        self.markerOut.writerow({'Time': rp.get_time(),
                                 'X': msg.pose.position.x,
                                 'Y': msg.pose.position.y,
                                 'Z': msg.pose.position.z,
                                 'qX': msg.pose.orientation.x,
                                 'qY': msg.pose.orientation.y,
                                 'qZ': msg.pose.orientation.z,
                                 'qW': msg.pose.orientation.w})




# Start node
saveAR = ar_listener(sys.argv[1], sys.argv[2], sys.argv[3])