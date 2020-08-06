#!/usr/bin/python

import rospy as rp
import threading
import math
import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

from pymavlink import mavutil

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

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

class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_node")

	# Create log file
	self.log_file = open("log.txt", 'a');

        # Set up Subscribers
        self.joy_sub = rp.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)

        # Set up Publishers

        # Connect to the Vehicle
        self.printAndLog('Connecting to Vehicle')
        self.vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

        # Variables
        self.cmds = None
        self.land = 0
        self.dsrm = 0
        self.arm = 0

        # Create thread for publisher
        self.rate = 5
        t = threading.Thread(target=self.send_commands)
        t.start()

        rp.spin()

    def printAndLog(self, msg):
	print(msg)
	self.log_file.write(msg)
	self.log_file.write("\n")

    def arm_and_takeoff_nogps(self, aTargetAltitude = -1.0):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.55
        SMOOTH_TAKEOFF_THRUST = 0.52

        self.printAndLog("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        #while not self.vehicle.is_armable:
         #   print(" Waiting for vehicle to initialise...")
          #  time.sleep(1)

        self.printAndLog("Arming motors")
        #  GUIDED_NOGPS mode is recommended by DroneKit
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS") 
        self.vehicle.armed = True

        while not self.vehicle.armed:
            self.printAndLog(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        self.printAndLog("Armed!")

        if aTargetAltitude > 0:
            print("Taking off!")

            thrust = DEFAULT_TAKEOFF_THRUST
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                print(" Altitude: %f  Desired: %f" %
                    (current_altitude, aTargetAltitude))
                if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                    print("Reached target altitude")
                    break
                elif current_altitude >= aTargetAltitude*0.6:
                    thrust = SMOOTH_TAKEOFF_THRUST
                self.set_attitude(thrust = thrust)
                time.sleep(0.2)

    def send_attitude_target(self, roll_angle = 0.0, pitch_angle = 0.0,
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
            yaw_angle = self.vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
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
        self.vehicle.send_mavlink(msg)

    def set_attitude(self, roll_angle = 0.0, pitch_angle = 0.0,
                     yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                     thrust = 0, duration = 0.001):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                 yaw_angle, yaw_rate, False,
                                 thrust)
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0, 0, 0, True, thrust)
	msg = "            actual roll: " + str(roll_angle) + " actual pitch: " + str(pitch_angle)
	self.printAndLog(msg)

    def joy_callback(self, data):
        self.cmds = Twist()

        # Joystick Controls
        self.cmds.linear.x  = data.axes[2]*10 #roll
        self.cmds.linear.y  = data.axes[3]*10 #pitch
        self.cmds.linear.z  = 0.1 * data.axes[1] + 0.5 #thrust
        self.cmds.angular.z = data.axes[4]*5 #yaw

        # Button Controls
        self.dsrm = data.buttons[0]
        self.land = data.buttons[1]
        self.arm = data.buttons[9]
	self.mag = data.axes[5]
	self.exit = data.buttons[2]

	msg = "arm: " + str(self.arm) + " disarm: " + str(self.dsrm) + " magnet: " + str(self.mag)
        self.printAndLog(msg)

    def engage_magnet(self):
	msg_hi = self.vehicle.message_factory.command_long_encode(
        	0, 0,   # target_system, target_command
        	mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
		0,
        	8,    # servo number
        	2006, # servo position
        	0, 0, 0, 0, 0)

    	msg_neut = self.vehicle.message_factory.command_long_encode(
        	0, 0,   # target_system, target_command
        	mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
		0,
        	8,    # servo number
        	1500, # servo position
        	0, 0, 0, 0, 0)

    	#send command
	self.vehicle.send_mavlink(msg_hi)
    	self.printAndLog("Magnet Engaged")
    	time.sleep(5)
    	self.vehicle.send_mavlink(msg_neut)
    	self.printAndLog("Magnet in Neutral")
    	self.printAndLog("complete")


    def disengage_magnet(self):
	msg_lo = self.vehicle.message_factory.command_long_encode(
        	0, 0,   # target_system, target_command
        	mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
		0,
        	8,    # servo number
        	982, # servo position
        	0, 0, 0, 0, 0)

    	msg_neut = self.vehicle.message_factory.command_long_encode(
        	0, 0,   # target_system, target_command
        	mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
		0,
        	8,    # servo number
        	1500, # servo position
        	0, 0, 0, 0, 0)

    	#send command
	self.vehicle.send_mavlink(msg_lo)
    	self.printAndLog("Magnet Disengaged")
    	time.sleep(5)
    	self.vehicle.send_mavlink(msg_neut)
    	self.printAndLog("Magnet in Neutral")
    	self.printAndLog("complete")



    def send_commands(self):
        self.printAndLog("Accepting Commands")
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.cmds is not None:

                 
                self.set_attitude(roll_angle = -self.cmds.linear.x, pitch_angle = -self.cmds.linear.y, yaw_angle = None, yaw_rate = self.cmds.angular.z, thrust = self.cmds.linear.z)

		msg = "thrust: " + str(self.cmds.linear.z) + " roll angle: " + str(self.cmds.linear.x) + " pitch angle: " + str(self.cmds.linear.y)
		self.printAndLog(msg)
                if self.dsrm > 0:
		    self.printAndLog("Disarming")
                    self.set_attitude(thrust = 0, duration = 8)
		    self.printAndLog("Disarm complete")
                if self.arm > 0:
                    self.printAndLog("Arming...")
                    self.arm_and_takeoff_nogps()
		if self.mag > 0:
		    self.printAndLog("Engaging Magnet")
		    self.engage_magnet()
		if self.mag < 0:
		    self.printAndLog("Disengaging Magnet")
		    self.disengage_magnet()
		if self.exit > 0:
		    self.printAndLog("Switched to manual controls")
            r.sleep()

# Start Node
magdrone = magdroneControlNode()
