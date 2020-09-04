#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy as rp

from dronekit import connect, VehicleMode

import time

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Vector3Stamped


# Connect to the Vehicle. 
print("\nConnecting to vehicle ")
vehicle = connect('/dev/serial0', wait_ready= True, baud=57600)

while True:
    roll = vehicle.attitude.roll
    pitch = vehicle.attitude.pitch 
    yaw = vehicle.attitude.yaw

    print(roll, pitch, yaw)


