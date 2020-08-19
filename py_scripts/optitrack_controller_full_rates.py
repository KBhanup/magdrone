#!/usr/bin/python

import rospy as rp
import threading
import math
import time

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil

from logbook import LogBook

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped


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


class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_node")

        # Create Controllers
        self.kp_z = 0.55
        self.kd_z = 0.33
        self.kp_y = 7.5
        self.kd_y = 6.5
        self.kp_x = 7.5
        self.kd_x = 6.5

        self.z_error = 0.0
        self.y_error = 0.0
        self.x_error = 0.0
        self.z_error_d = 0.0
        self.y_error_d = 0.0
        self.x_error_d = 0.0

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
        # Empty Commands
        # self.cmds.linear.x = 0   # roll
        # self.cmds.linear.y = 0   # pitch
        # self.cmds.angular.z = 0  # yaw
        # self.linear_z_cmd = 0   # thrust
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

        # Position conversions where the reported position is in terms of the camera frame
        # z-error = x-tag - z_des = y-camera
        # y-error = y-tag - y_des = x-camera
        # x-error = z-tag - x_des = z-camera
        self.z_error = self.z_des - data.pose.position.z
        self.y_error = self.y_des - data.pose.position.y
        self.x_error = data.pose.position.x - self.x_des

    def rate_callback(self, data):
        self.z_error_d = -data.twist.linear.z
        self.y_error_d = -data.twist.linear.y
        self.x_error_d = data.twist.linear.x

    def joy_callback(self, data):
        # Empty Command
        self.cmds = Twist()

        # Joystick Controls
        self.cmds.linear.x = data.axes[2]*10  # roll
        self.cmds.linear.y = data.axes[3]*10  # pitch
        self.cmds.angular.z = data.axes[0]*10  # yaw

        # Button Controls
        self.dsrm = data.buttons[0]
        self.land = data.buttons[1]
        self.arm = data.buttons[9]
        self.mag = data.axes[5]
        self.exit = data.buttons[2]

    def send_commands(self):
        print("Accepting Commands")
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.cmds is not None:
                # Generate commands
                uZ = self.kp_z * self.z_error + self.kd_z * self.z_error_d
                uY = self.kp_y * self.y_error + self.kd_y * self.y_error_d
                uX = self.kp_x * self.x_error + self.kd_x * self.x_error_d

                self.linear_z_cmd = self.clipCommand(uZ + 0.5, 0.65, 0.35)
                self.linear_y_cmd = self.clipCommand(uY, 7.5, -7.5)
                self.linear_x_cmd = self.clipCommand(uX, 7.5, -7.5)

                self.set_attitude(roll_angle=self.linear_y_cmd, pitch_angle=-self.linear_x_cmd,
                                  yaw_angle=None, yaw_rate=-self.cmds.angular.z, use_yaw_rate=True, thrust=self.linear_z_cmd, duration=1.0/self.rate)

                if self.arm > 0:
                    self.log_book.printAndLog("Arming...")
                    self.arm_and_takeoff_nogps()
                if self.arm > 0:
                    self.log_book.printAndLog("Arming...")
                    self.arm_and_takeoff_nogps()
                if self.exit == 0:
                    msg = str(self.z_error) + "\t" + str(self.linear_z_cmd) + "\t" + str(self.y_error) + \
                        "\t" + str(self.linear_y_cmd) + "\t" + \
                        str(self.x_error) + "\t" + str(self.linear_x_cmd)
                    self.log_book.justLog(msg)
            r.sleep()


# Start Node
magdrone = magdroneControlNode()
