from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

# Connect to the Vehicle
print('Connecting to vehicle on')
vehicle = connect('/dev/serial0', wait_ready= True, baud=57600)

def arm_and_takeoff_nogps(aTargetAltitude = -1.0):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.55
    SMOOTH_TAKEOFF_THRUST = 0.52

    #print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    #while not vehicle.is_armable:
    #   print(" Waiting for vehicle to initialise...")
    #    time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Armed!")
    
    if aTargetAltitude > 0:
        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f" %
                (current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude*0.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            set_attitude(thrust = thrust)
            time.sleep(0.2)

#everything up to here is copied from dummyControl.py

#next section copied from https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Set up velocity mappings
# velocity_x > 0 => fly North
# velocity_x < 0 => fly South
# velocity_y > 0 => fly East
# velocity_y < 0 => fly West
# velocity_z < 0 => ascend
# velocity_z > 0 => descend
SOUTH=-2
UP=-0.5   #NOTE: up is negative!

#Arm, then fly south and up.
arm_and_takeoff_nogps()
send_ned_velocity(SOUTH,0,UP,DURATION)