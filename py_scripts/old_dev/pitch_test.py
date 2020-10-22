from dronekit import *
from pymavlink import mavutil #for command message definitions 
import time
import math 

#connect to the vehicle
vehicle = connect('/dev/serial0', wait_ready= True,baud=57600)


def set_attitude_initial(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0):
	
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

	send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
	start = time.time()
	while time.time() - start < duration:
		send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)	
	time.sleep(0.1)

	send_attitude_target(0, 0, 0, 0, True, thrust)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5):
	if yaw_angle is None:
		yaw_angle = vehicle.attitude.yaw
	msg = vehicle.message_factory.set_attitude_target_encode(
		0, #time_boot_ms
		1, #target system
		1, #target component 
		0b00000000 if use_yaw_rate else 0b00000100,
		to_quaternion(roll_angle, pitch_angle, yaw_angle),
		0, #body roll rate in radian
		0, #body pitch rate in radian
		math.radians(yaw_rate), #body yaw rate in rad/s
		thrust #thrust
	)
	vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0):
	
	send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
	start = time.time()
	while time.time() - start < duration:
		send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)	
	time.sleep(0.1)

	send_attitude_target(0, 0, 0, 0, True, thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
	t0 = math.cos(math.radians(yaw*0.5))
	t1 = math.sin(math.radians(yaw*0.5))
	t2 = math.cos(math.radians(roll*0.5))
	t3 = math.sin(math.radians(roll*0.5))
	t4 = math.cos(math.radians(pitch*0.5))
	t5 = math.sin(math.radians(pitch*0.5))

	w = t0 * t2 * t4 + t1 * t3 * t5
	x = t0 * t3 * t4 - t1 * t2 * t5
	y = t0 * t2 * t5 + t1 * t3 * t4
	z = t1 * t2 * t4 - t0 * t3 * t5

	return [w, x, y, z]

#############################

set_attitude_initial(thrust = 0.5, duration = 3)

print("move forward")
set_attitude(pitch_angle = -5, thrust = 0.5, duration = 10)

print("setting LAND mode")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)

print("Close vehicle object")
vehicle.close()

print("Completed")


