#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy as rp

from dronekit import connect, VehicleMode

import threading
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Vector3Stamped

class magdroneControlNode():

    def __init__(self):
        # Set up Subscribers
        self.joy_sub = rp.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
    
        # Set up Publishers
        self.rpy_pub = rp.Publisher("/rpy", Vector3Stamped, queue_size=1)

        # Connect to the Vehicle. 
        print("\nConnecting to vehicle ")
        vehicle = connect('/dev/serial0', wait_ready= True, baud=57600)

        # Create thread for publisher
        self.rate = 30
        t = threading.Thread(target=self.rpy)
        t.start()

    def rpy(self):
        rp.loginfo("Accepting Commands")
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.arm == 0:        
                print('starting rpy')
                roll = vehicle.attitude.roll
                pitch = vehicle.attitude.pitch 
                yaw = vehicle.attitude.yaw

                # Publish setpoint
                self.rpy = Vector3Stamped()
                rpy.header.stamp = rp.Time.now()
                rpy.vector.roll = roll
                rpy.vector.pitch = pitch
                rpy.vector.yaw = yaw
                self.rpy_pub.publish(rpy)

    def joy_callback(self, data):
        # Empty Command
        self.cmds = Twist()

        # Joystick Controls

        # Button Controls
        self.arm = data.buttons[9]

magdroneControlNode()  


