from dronekit import *

#connect to vehicle
vehicle = connect('/dev/serial0', wait_ready= True,baud=57600)

def arm_and_takeoff(aTargetAltitude):
	#arms vehicle and flies to aTargetAltitude

	print "Basic prearm checks"
	#don't try to arm until autopilot is ready
	while not vehicle.is_armable:
		print "Waiting for vehicle to initialize..."
		time.sleep(1)

	print "Arming Motors"
	#copter should arm in GUIDED mode
	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True

	#confirm vehicle armed before attempting to take off
	while not vehicle.armed:
		print "Waiting for arming ... "
		time.sleep(1)

	print "Taking off!"
	vehicle.simple_takeoff(aTargetAltitude) #take off to target altitude

	while True:
		print "Altitude: ", vehicle.location.global_relative_frame.alt
		#break and return from function just below target altitude
		if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
			print "reached target altitude"
			break
		time.sleep(1)

arm_and_takeoff(20)
