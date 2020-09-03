#!/usr/bin/python

import rospy as rp
import threading
import math
import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil


# Connect to the Vehicle
print('Connecting to vehicle on')
vehicle = connect('/dev/serial0', wait_ready= True, baud=57600)

print(" Attitude: %s" % vehicle.attitude)

# Attempt to make IMU listener
vehicle.add_message_listener(‘HIGHRES_IMU’,receivedImu)

def receivedImu(self, name, imuMsg):
    print(imuMsg)


