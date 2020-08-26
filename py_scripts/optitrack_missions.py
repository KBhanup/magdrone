#!/usr/bin/python

import rospy as rp
import numpy as np
import threading
import math
import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

from logbook import LogBook

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped

def to_rpy(qw, qx, qy, qz):
    r = np.arctan2(2 * (qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))

    sinP = 2 * (qw*qy - qz*qx)
    if np.abs(sinP) > 1:
        p = np.sign(sinP) * np.pi / 2
    else:
        p = np.arcsin(sinP)

    y = np.arctan2(2 * (qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    return np.rad2deg([r, p, y])

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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

def opti_to_drone(x_error,y_error,yaw_angle):
    x_error_d = (x_error * math.cos(math.radians(yaw_angle))) - (y_error * math.sin(math.radians(yaw_angle)))
    y_error_d = (x_error * math.sin(math.radians(yaw_angle))) + (y_error * math.cos(math.radians(yaw_angle)))
    return [x_error_d, y_error_d]

def mission_one(x_error,y_error,thrust_error):
    struct_x = 0.04
    struct_y = 0.02
    struct_z = 2.08

    desired_positions = [-1.5, -1.4, -1.3, -1.2, -1.1, -1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3]

    self.x_des = struct_x
    self.y_des = struct_y
    self.z_des = desired_positions[x] + struct_z
    
    if -0.05 < self.z_des < 0.05 & -0.05 < self.x_des < 0.05 & -0.05 < self.y_des < 0.05 == True:
            self.z_des = desired_positions[x+1] + struct_z

    if self.z_des = 0.0:
        self.engage_magnet()

def mission_two(x_error,y_error,thrust_error):
    self.x_des = 0.0
    self.y_des = -1.8
    self.z_des = 1.0


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
    	self.log_book.printAndLog("Magnet Engaged")
    	time.sleep(5)
    	self.vehicle.send_mavlink(msg_neut)
    	self.log_book.printAndLog("Magnet in Neutral")
    	self.log_book.printAndLog("complete")


class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_node")

        # Create Controllers
        self.kp_z = 0.45
        self.kd_z = 0.3
        self.kp_y = 10.5
        self.kd_y = 8.5
        self.kp_x = 10.5
        self.kd_x = 8.5
        self.kp_yaw = 0.15

        self.z_error = 0.0
        self.y_error = 0.0
        self.x_error = 0.0
        self.z_error_d = 0.0
        self.y_error_d = 0.0
        self.x_error_d = 0.0
        self.yaw_error = 0.0

        # Make a global variable for the current yaw position to be used for pose and rate transormations
        self.yaw_position = 0.0

        # Create log file
        self.log_book = LogBook("test_flight")

        # Set up Subscribers
        self.pose_sub = rp.Subscriber(
            "/opti_state/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.pose_sub = rp.Subscriber(
            "/opti_state/rates", TwistStamped, self.rate_callback, queue_size=1)
        self.joy_sub = rp.Subscriber(
            "/joy", Joy, self.joy_callback, queue_size=1)

        # Labeling the log file by controller
        self.log_book.printAndLog('Running optitrack_controller.py')

        # Connect to the Vehicle
        #self.log_book.printAndLog('Connecting to Vehicle')
        print('Connecting to Vehicle')
        self.vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

        # Variables
        self.cmds = None
        self.linear_z_cmd = 0.0
        self.linear_y_cmd = 0.0
        self.land = 0
        self.dsrm = 0
        self.arm = 0
        self.exit = 0

        # Create thread for publisher
        self.rate = 30
        t = threading.Thread(target=self.send_commands)
        t.start()

        rp.spin()

    def clipCommand(self, cmd, upperBound, lowerBound):
        if cmd < lowerBound:
            cmd = lowerBound
        elif cmd > upperBound:
            cmd = upperBound

        return cmd

    def arm_and_takeoff_nogps(self, aTargetAltitude=-1.0):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.55
        SMOOTH_TAKEOFF_THRUST = 0.52

        #self.log_book.printAndLog("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        # while not self.vehicle.is_armable:
        #   print(" Waiting for vehicle to initialise...")
        #  time.sleep(1)

        #self.log_book.printAndLog("Arming motors")
        #  GUIDED_NOGPS mode is recommended by DroneKit
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            #self.log_book.printAndLog(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        # self.log_book.printAndLog("Armed!")
        print('Armed')

        if aTargetAltitude > 0:
            print("Taking off!")

            thrust = DEFAULT_TAKEOFF_THRUST
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                print(" Altitude: %f  Desired: %f" %
                      (current_altitude, aTargetAltitude))
                # Trigger just below target alt.
                if current_altitude >= aTargetAltitude*0.95:
                    print("Reached target altitude")
                    break
                elif current_altitude >= aTargetAltitude*0.6:
                    thrust = SMOOTH_TAKEOFF_THRUST
                self.set_attitude(thrust=thrust)
                time.sleep(0.2)

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                             thrust=0.5):
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
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                     thrust=0, duration=0.05):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                                  yaw_angle, yaw_rate, use_yaw_rate,
                                  thrust)
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                      yaw_angle, yaw_rate, use_yaw_rate,
                                      thrust)
            time.sleep(duration)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0, 0, 0, True, thrust)

    def pose_callback(self, data):

        """
        + z error = + thrust
        - z error = - thrust
        + y error = - roll
        - y error = + roll
        + x error = + pitch
        - x error = - pitch
        """
        # Define the desired position
        self.z_des = 1.0  # thrust
        self.y_des = 0.0  # roll
        self.x_des = 0.0  # pitch
        self.yaw_des = 180.0 # yaw in degrees

        #Get orientation data 
        qw = data.pose.orientation.w
        qx = data.pose.orientation.x
        qy = data.pose.orientation.y
        qz = data.pose.orientation.z

        orientation = to_rpy(qw, qx, qy, qz)

        self.yaw_position = -orientation[2] 

        # Position conversions where the reported position is in terms of the camera frame
        # z-error = x-tag - z_des = y-camera
        # y-error = y-tag - y_des = x-camera
        # x-error = z-tag - x_des = z-camera
        self.z_error = self.z_des - data.pose.position.z
        self.y_error = self.y_des - data.pose.position.y
        self.x_error = self.x_des - data.pose.position.x
        yaw_diff = self.yaw_des -  self.yaw_position

        # Translate error from optitrack frame to drone body frame
        drone_error = opti_to_drone(self.x_error,self.y_error,self.yaw_position)

        # Update x and y error 
        self.x_error = drone_error[0]
        self.y_error = drone_error[1]

        if yaw_diff > 180.0:
            self.yaw_error = yaw_diff - 360.0
        else:
            self.yaw_error = yaw_diff

    def rate_callback(self, data):
        self.z_error_d = -data.twist.linear.z
        self.y_error_d = -data.twist.linear.y
        self.x_error_d = -data.twist.linear.x

        # Translate error rate from optitrack frame to drone body frame
        drone_error_rate = opti_to_drone(self.x_error_d,self.y_error_d,self.yaw_position)

        # Update x and y error rate
        self.x_error_d = drone_error_rate[0]
        self.y_error_d = drone_error_rate[1]

    def joy_callback(self, data):
        # Empty Command
        self.cmds = Twist()

        # Joystick Controls
        self.cmds.linear.x = data.axes[2]*10  # roll
        self.cmds.linear.y = data.axes[3]*10  # pitch
        self.cmds.angular.z = data.axes[0]*10  # yaw

        # Button Controls
        self.mission_one = data.buttons[0]
        self.mission_two = data.buttons[1]
        self.arm = data.buttons[9]
        self.mag = data.axes[5]
        self.exit = data.buttons[2]

    def send_commands(self):
        print("Accepting Commands")
        if self.arm > 0:
            self.log_book.printAndLog("Arming...")
            self.arm_and_takeoff_nogps()
        
        if self.exit == 0:
            msg = str(self.z_error) + "\t" + str(self.linear_z_cmd) + "\t" + str(self.y_error) + "\t" + str(self.linear_y_cmd) + "\t" + str(self.x_error) + "\t" + str(self.linear_x_cmd) + "\t" + str(self.yaw_error) + "\t" + str(self.angular_z_cmd) + "\t" + str(self.x_error_d) + "\t" + str(self.y_error_d)
            self.log_book.justLog(msg)
            #print("x position error: " + str(self.x_error) + " y position error: " + str(self.y_error) + "yaw position error: " + str(self.yaw_error))
            #print("pitch cmd: " + str(self.linear_x_cmd) + " roll cmd: " + str(-self.linear_y_cmd) + " yaw cmd: " + str(self.angular_z_cmd))

        if self.mission_one > 0:
            mission_one(self.x_error,self.y_error,self.z_error)
            # Generate commands
            uZ = self.kp_z * self.z_error + self.kd_z * self.z_error_d
            uY = self.kp_y * self.y_error + self.kd_y * self.y_error_d
            uX = self.kp_x * self.x_error + self.kd_x * self.x_error_d
            uW = self.kp_yaw * self.yaw_error

            self.linear_z_cmd = self.clipCommand(uZ + 0.5, 0.65, 0.35)
            self.linear_y_cmd = self.clipCommand(uY, 7.5, -7.5)
            self.linear_x_cmd = self.clipCommand(uX + 0.45, 7.5, -7.5)
            self.angular_z_cmd = self.clipCommand(uW, 5, -5)

            self.set_attitude(roll_angle=-self.linear_y_cmd, pitch_angle=-self.linear_x_cmd, yaw_angle=None, yaw_rate=self.angular_z_cmd, use_yaw_rate=True, thrust=self.linear_z_cmd, duration=1.0/self.rate)

        if self.mission_two > 0:
            mission_two(self.x_error,self.y_error,self.z_error)
            # Generate commands
            uZ = self.kp_z * self.z_error + self.kd_z * self.z_error_d
            uY = self.kp_y * self.y_error + self.kd_y * self.y_error_d
            uX = self.kp_x * self.x_error + self.kd_x * self.x_error_d
            uW = self.kp_yaw * self.yaw_error

            self.linear_z_cmd = self.clipCommand(uZ + 0.5, 0.65, 0.35)
            self.linear_y_cmd = self.clipCommand(uY, 7.5, -7.5)
            self.linear_x_cmd = self.clipCommand(uX + 0.45, 7.5, -7.5)
            self.angular_z_cmd = self.clipCommand(uW, 5, -5)

            self.set_attitude(roll_angle=-self.linear_y_cmd, pitch_angle=-self.linear_x_cmd, yaw_angle=None, yaw_rate=self.angular_z_cmd, use_yaw_rate=True, thrust=self.linear_z_cmd, duration=1.0/self.rate)


        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.cmds is not None:

                
                if self.arm > 0:
                    self.log_book.printAndLog("Arming...")
                    self.arm_and_takeoff_nogps()
            r.sleep()

# Start Node
magdrone = magdroneControlNode()
