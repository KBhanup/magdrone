#begin by importing everything
from dronekit import *
from pymavlink import mavutil #for command message definitions 
import time
import math 

#connect to vehicle
vehicle = connect('/dev/serial0', wait_ready= True,baud=57600)

def send_frame_velocity(vel_x, vel_y, vel_z, duration):

	msg = vehicle.message_factory.mav_frame_local_ned_encode(
	
