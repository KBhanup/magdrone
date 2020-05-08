#!/usr/bin/python

import rospy as rp
import threading

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

from pymavlink import mavutil

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_node")

        # Set up Subscribers
        self.joy_sub = rp.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        # Set up Publishers

        # Initialize the dronekit stuff here

        # Variables
        self.cmds = None
        self.land = None
        # etc...

        # Create thread for publisher
        self.rate = 10
        t = threading.Thread(target=self.send_commands)
        t.start()

        rp.spin()

    def joy_callback(self, data):
        self.cmds = Twist()

        # Make sure the order is correct here
        self.cmds.linear.x  = data.axes[0]
        self.cmds.linear.y  = data.axes[1]
        self.cmds.linear.z  = data.axes[2]
        self.cmds.angular.z = data.axes[3]

        # Add anything else you need here
        self.land = data.buttons[0]
    
    def send_commands(self):
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.cmds is not None:
                # Send the commands to dronekit here
                # I am printing them just to make sure it works
                print(self.cmds.linear.x)
                print(self.cmds.linear.y)
                print(self.cmds.linear.z)
                print(self.cmds.angular.z)
            r.sleep()

# Start Node
magdroneC = magdroneControlNode()