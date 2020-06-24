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

def magnet_test():
    print("testing magnet")

    msg_neut = vehicle.message_factory.command_long_encode(
        0, 0,   # target_system, target_command
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
	0,
        8,    # servo number
        1500, # servo position
        0, 0, 0, 0, 0)

    msg_low = vehicle.message_factory.command_long_encode(
        0, 0,   # target_system, target_command
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
	0,
        8,    # servo number
        982, # servo position
        0, 0, 0, 0, 0)

    msg_hi = vehicle.message_factory.command_long_encode(
        0, 0,   # target_system, target_command
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
	0,
        8,    # servo number
        2006, # servo position
        0, 0, 0, 0, 0)

    #send command
    vehicle.send_mavlink(msg_hi)
    print("high mode")
    time.sleep(5)
    vehicle.send_mavlink(msg_low)
    print("disengaging")    
    time.sleep(5)
    vehicle.send_mavlink(msg_neut)
    print("complete")

# run it 
magnet_test()
