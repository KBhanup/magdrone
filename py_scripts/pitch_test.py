from dronekit import *
from pymavlink import mavutil #for command message definitions 
import time
import math 

##skipping some argparse stuff, tbd if that will backfire

#connect to the vehicle
vehicle = connect('/dev/serial0', wait_ready= True,baud=57600)

def arm_and_takeoff_nogps(aTargetAltitude):
	"""
	Arms vehicle and flies to aTargetAltitude without GPS data.
	"""
	
	###CONSTANTS###
	DEFAULT_TAKEOFF_THRUST= 0.6
	SMOOTH_TAKEOFF_THRUST = 0.55

	print("basic pre-arm checks")
	#prevents user from arming until the autopilot is ready
	while not vehicle.is_armable:
		print("waiting for vehicle to initialize...")
		time.sleep(1)

	print("arming motors")
	#copter should arm without GPS signal
	vehicle.mode = VehicleMode("GUIDED_NOGPS")
	vehicle.armed = True
	
	while not vehicle.armed:
		print("waiting for arming...")
		vehicle.arm = True
		time.sleep(1)

	print("Taking off!")

	thrust = DEFAULT_TAKEOFF_THRUST
	while True:
		current_altitude = vehicle.location.global_relative_frame.alt
		print("altitude: %f Desired: %f" %
			(current_altitude, aTargetAltitude))
		if current_altitude >= aTargetAltitude*0.95:
			print("reached target altitude")
			break
		elif current_altitude >= aTargetAltitude*0.6:
			thrust=SMOOTH_TAKEOFF_THRUST
		set_attitude(thrust = thrust)
		time.sleep(0.2)

def
