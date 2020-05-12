#!/usr/bin/python

import rospy as rp
import threading

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

from pymavlink import mavutil

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_node")

        # Set up Subscribers
        self.joy_sub = rp.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        # Set up Publishers
	self.joy_pub = rp.Publisher("/joy", sensor_msgs.msg.String, queue_size=1)
 
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

	    print("Basic pre-arm checks")
	    # Don't let the user try to arm until autopilot is ready
	    # If you need to disable the arming check,
	    # just comment it with your own responsibility.
	    while not vehicle.is_armable:
	        print(" Waiting for vehicle to initialise...")
	        time.sleep(1)

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

	def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
	                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
	                         thrust = 0.5):
	    """
	    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
	                  When one is used, the other is ignored by Ardupilot.
	    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
	            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
	            the code for maintaining current altitude.
	    """
	    if yaw_angle is None:
	        # this value may be unused by the vehicle, depending on use_yaw_rate
	        yaw_angle = vehicle.attitude.yaw
	    # Thrust >  0.5: Ascend
	    # Thrust == 0.5: Hold the altitude
	    # Thrust <  0.5: Descend
	    msg = vehicle.message_factory.set_attitude_target_encode(
	        0, # time_boot_ms
	        1, # Target system
	        1, # Target component
	        0b00000000 if use_yaw_rate else 0b00000100,
	        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
	        0, # Body roll rate in radian
	        0, # Body pitch rate in radian
	        math.radians(yaw_rate), # Body yaw rate in radian/second
	        thrust  # Thrust
	    )
	    vehicle.send_mavlink(msg)
	
	def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
	                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
	                 thrust = 0.5, duration = 0):
	    """
	    Note that from AC3.3 the message should be re-sent more often than every
	    second, as an ATTITUDE_TARGET order has a timeout of 1s.
	    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
	    The code below should work on either version.
	    Sending the message multiple times is the recommended way.
	    """
	    send_attitude_target(roll_angle, pitch_angle,
	                         yaw_angle, yaw_rate, False,
	                         thrust)
	    start = time.time()
	    while time.time() - start < duration:
	        send_attitude_target(roll_angle, pitch_angle,
	                             yaw_angle, yaw_rate, False,
	                             thrust)
	        time.sleep(0.1)
	    # Reset attitude, or it will persist for 1s more due to the timeout
	    send_attitude_target(0, 0,
	                         0, 0, True,
	                         thrust)

	def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
	    """
	    Convert degrees to quaternions
	    """
	    t0 = math.cos(math.radians(yaw * 0.5))
	    t1 = math.sin(math.radians(yaw * 0.5))
	    t2 = math.cos(math.radians(roll * 0.5))
	    t3 = math.sin(math.radians(roll * 0.5))
	    t4 = math.cos(math.radians(pitch * 0.5))
	    t5 = math.sin(math.radians(pitch * 0.5))
	
	    w = t0 * t2 * t4 + t1 * t3 * t5
	    x = t0 * t3 * t4 - t1 * t2 * t5
	    y = t0 * t2 * t5 + t1 * t3 * t4
	    z = t1 * t2 * t4 - t0 * t3 * t5

	    return [w, x, y, z]

        # Variables
        self.cmds = None
        self.land = None
	self.dsrm = None
        # etc...

        # Create thread for publisher
        self.rate = 10
        t = threading.Thread(target=self.send_commands)
        t.start()

        rp.spin()

    def joy_callback(self, data):
        self.cmds = Twist()

        # Make sure the order is correct here
        self.cmds.linear.x  = data.axes[2]*5 #roll
        self.cmds.linear.y  = data.axes[3]*5 #pitch
        self.cmds.linear.z  = data.axes[1]+0.52 #thrust
        self.cmds.angular.z = data.axes[0]*5 #yaw

        # Add anything else you need here
        self.dsrm = data.buttons[0]
    	self.land = data.buttons[1]

    def send_commands(self):
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.cmds is not None:
                # Send the commands to dronekit here
		set_attitude(roll_angle = self.cmds.linear.x, pitch_angle = self.cmds.linear.y,
                 yaw_angle = None, yaw_rate = self.cmds.angular.z, thrust = self.cmds.linear.z)
	    elif self.dsrm == 1:
		set_attitude(thrust = 0, duration = 5)
	    elif self.land == 1:
        	print("Setting LAND mode...")
        	vehicle.mode = VehicleMode("LAND")
        	time.sleep(1)
        	break
    	    else:
          	set_attitude(thrust = 0.5, duration = 1)
            r.sleep()

# Start Node
magdroneC = magdroneControlNode()
