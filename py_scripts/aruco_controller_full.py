#!/usr/bin/python

import rospy as rp
import threading
import math
import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

from pymavlink import mavutil

from pid import PIDcontroller

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Vector3Stamped

def to_rpy(qw, qx, qy, qz):
    r = math.atan2(2 * (qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))

    sinP = 2 * (qw*qy - qz*qx)
    if abs(sinP) > 1:
        p = math.copysign(1.0, sinP) * math.pi / 2
    else:
        p = math.asin(sinP)

    y = math.atan2(2 * (qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    return [math.degrees(r), math.degrees(p), math.degrees(y)]

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

def tag_to_drone(x_error, y_error, yaw_angle):
    x_error_d = (x_error * math.cos(math.radians(yaw_angle))) - (y_error * math.sin(math.radians(yaw_angle)))
    y_error_d = (x_error * math.sin(math.radians(yaw_angle))) + (y_error * math.cos(math.radians(yaw_angle)))
    return [x_error_d, y_error_d]


class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_node")

        # Create PID Controller
        self.pid_z = PIDcontroller(1.0, 0.0, 17.5, 3)
        self.pid_y = PIDcontroller(9.5, 0.0, 200.0, 30)
        self.pid_x = PIDcontroller(9.5, 0.0, 200.0, 30)
        self.pid_w = PIDcontroller(0.15, 0.0, 0.0, 30)

        # Create log file
        self.log_file = open("log.txt", 'a')

        # Set up Subscribers
        self.pose_sub = rp.Subscriber(
            "/tf", PoseStamped, self.pose_callback, queue_size=1)
        self.joy_sub = rp.Subscriber(
            "/joy", Joy, self.joy_callback, queue_size=1)

        # Set up Publishers
        self.setpoint_pub = rp.Publisher("/setpoint", Vector3Stamped, queue_size=1)
        self.command_pub = rp.Publisher("/commands", TwistStamped, queue_size=1)

        # Connect to the Vehicle
        self.printAndLog('Connecting to Vehicle')
        self.vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

        # Variables
        self.cmds = None
        self.land = 0
        self.dsrm = 0
        self.arm = 0
        self.exit = 0

        # Create thread for publisher
        self.rate = 20
        t = threading.Thread(target=self.send_commands)
        t.start()

        rp.spin()

    def printAndLog(self, msg):
        print(msg)
        self.log_file.write(msg)
        self.log_file.write("\n")

    def arm_and_takeoff_nogps(self, aTargetAltitude=-1.0):
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
        # while not self.vehicle.is_armable:
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
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0, 0, 0, True, thrust)

    def pose_callback(self, data):
        self.cmds = Twist()

        # Get current pose wrt Tag
        qw = data.transforms.transform.rotation.w
        qx = data.transforms.transform.rotation.x
        qy = data.transforms.transform.rotation.y
        qz = data.transforms.transform.rotation.z
        orientation = to_rpy(qw, qx, qy, qz)

        # Drone body system is Front-Left-Down
        w_drone = -orientation[2]
        x_drone = data.transforms.transform.translation.x
        y_drone = data.transforms.transform.translation.y
        z_drone = data.transforms.transform.translation.z

        # Update yaw
        self.yaw_position = w_drone

        # Set desired position
        des_position = [0,0,1] #x, y, and z

        # Publish setpoint
        setpoint = Vector3Stamped()
        setpoint.header.stamp = rp.Time.now()
        setpoint.vector.x = des_position[0]
        setpoint.vector.y = des_position[1]
        setpoint.vector.z = des_position[2]
        self.setpoint_pub.publish(setpoint)

	    """
	    + z error = + thrust
    	- z error = - thrust
	    + y error = - roll
	    - y error = + roll
	    + x error = + pitch
	    - x error = - pitch 		
	    """

        # Position conversions where the reported position is in terms of the camera frame
	    # z-error = x-tag - z_des = y-camera
	    # y-error = y-tag - y_des = x-camera
	    # x-error = z-tag - x_des = z-camera

        # Calculate error
        dx = des_position[0] - x_drone
        dy = des_position[1] - y_drone
        dz = des_position[2] - z_drone
        dw = 90.0 - w_drone

        # Translate error from optitrack frame to drone body frame
        drone_error = tag_to_drone(dx, dy, w_drone)

        self.x_error = drone_error[0]
        self.y_error = drone_error[1]
        self.z_error = dz

        if dw > 180.0:
            self.w_error = dw - 360.0
        else:
            self.w_error = dw


    def opti_callback(self, data):
        self.lastOnline = time.time()
    
    def clipCommand(self, cmd, upperBound, lowerBound):
        if cmd < lowerBound:
            cmd = lowerBound
        elif cmd > upperBound:
            cmd = upperBound

        return cmd

    def send_commands(self):
        rp.loginfo("Accepting Commands")
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.arm > 0:
                self.printAndLog("Arming...")
                self.arm_and_takeoff_nogps()
            if self.exit > 0:
                self.printAndLog("Switched to manual controls")
            if self.cmds is not None and self.vehicle.armed:   
                # Generate commands
                uZ = self.kp_z * self.z_error + self.kd_z * self.z_error_d
                uY = self.kp_y * self.y_error + self.kd_y * self.y_error_d
                uX = self.kp_x * self.x_error + self.kd_x * self.x_error_d
                uW = self.kp_w * self.w_error

                linear_z_cmd  = self.clipCommand(uZ + 0.5, 0.65, 0.35)
                linear_y_cmd  = self.clipCommand(uY - 1.3, 7.5, -7.5)
                linear_x_cmd  = self.clipCommand(uX + 0.45, 7.5, -7.5)
                angular_z_cmd = self.clipCommand(uW, 5, -5)

                cmd = TwistStamped()
                cmd.header.stamp = rp.Time.now()
                cmd.twist.angular.x = -linear_y_cmd
                cmd.twist.angular.y = -linear_x_cmd
                cmd.twist.angular.z =  angular_z_cmd
                cmd.twist.linear.z  =  linear_z_cmd
                self.command_pub.publish(cmd)
                
                if (time.time() - self.lastOnline < 0.5):
                    self.set_attitude(roll_angle = -linear_y_cmd,
                                      pitch_angle = -linear_x_cmd,
                                      yaw_angle = None,
                                      yaw_rate = angular_z_cmd, use_yaw_rate = True, 
                                      thrust = linear_z_cmd,
                                      duration=1.0/self.rate)
                else:
                    self.set_attitude(roll_angle = 0.0,
                                      pitch_angle = 0.0,
                                      yaw_angle = None,
                                      yaw_rate = 0.0, use_yaw_rate = True, 
                                      thrust = 0.5,
                                      duration=1.0/self.rate)
                    rp.logwarn("Connection Lost!!! Hovering")
            r.sleep()

# Start Node
magdrone = magdroneControlNode()