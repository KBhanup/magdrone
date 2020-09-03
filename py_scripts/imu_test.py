#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py: 
Demonstrates how to get and set vehicle state and parameter information, 
and how to observe vehicle attribute (state) changes.
Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
import rospy as rp

from dronekit import connect, VehicleMode
import time
from geometry_msgs.msg import Vector3Stamped

def __init__():
    # Set up Publishers
    rpy_pub = rp.Publisher(
        "/rpy", Vector3Stamped, queue_size=1)

    # Connect to the Vehicle. 
    print("\nConnecting to vehicle ")
    vehicle = connect('/dev/serial0', wait_ready= True, baud=57600)

    roll = vehicle.attitude.roll
    pitch = vehicle.attitude.pitch 
    yaw = vehicle.attitude.yaw

    # Publish setpoint
    rpy = Vector3Stamped()
    rpy.header.stamp = rp.Time.now()
    rpy.vector.roll = roll
    rpy.vector.pitch = pitch
    rpy.vector.yaw = yaw
    rpy_pub.publish(rpy)

__init__()


