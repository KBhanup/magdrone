#!/usr/bin/python

import rospy as rp
import tf

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

def quatMultiply(p, q):
    w = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3]
    x = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2]
    y = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1]
    z = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]
    
    return [w, x, y, z]

def tag_to_drone(x_error, y_error, yaw_angle):
    x_error_d = (x_error * math.cos(math.radians(yaw_angle))) - \
        (y_error * math.sin(math.radians(yaw_angle)))
    y_error_d = (x_error * math.sin(math.radians(yaw_angle))) + \
        (y_error * math.cos(math.radians(yaw_angle)))
    return [x_error_d, y_error_d]

class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_aruco")

        # Connect to the Vehicle
        rp.loginfo('Connecting to Vehicle')
        self.vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

        # Set up tf listener
        self.tf_listener = tf.TransformListener()

        # Set up Subscribers
        self.joy_sub = rp.Subscriber(
            "/joy", Joy, self.joy_callback, queue_size=1)

        # Set up Publishers
        self.setpoint_pub = rp.Publisher(
            "/setpoint", Vector3Stamped, queue_size=1)
        self.command_pub = rp.Publisher(
            "/commands", TwistStamped, queue_size=1)

        # Create PID Controller
        self.pid_z = PIDcontroller(1.0, 0.0, 17.5, 3)
        self.pid_y = PIDcontroller(9.5, 0.0, 200.0, 30)
        self.pid_x = PIDcontroller(9.5, 0.0, 200.0, 30)
        self.pid_w = PIDcontroller(0.15, 0.0, 0.0, 30)

        # Variables
        self.arm = 0
        self.engage_controller = False

        # Create thread for publisher
        self.rate = 30
        t = threading.Thread(target=self.send_commands)
        t.start()

        rp.spin()

    '''
        DroneKit Functions
    '''

    def arm_and_takeoff_nogps(self, aTargetAltitude=-1.0):
        """
        Arms vehicle and fly to aTargetAltitude without GPS data.
        """

        ##### CONSTANTS #####
        DEFAULT_TAKEOFF_THRUST = 0.55
        SMOOTH_TAKEOFF_THRUST = 0.52

        rp.loginfo("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        # If you need to disable the arming check,
        # just comment it with your own responsibility.
        while not self.vehicle.is_armable:
            rp.loginfo(" Waiting for vehicle to initialise...")
            time.sleep(1)

        rp.loginfo("Arming motors")
        #  GUIDED_NOGPS mode is recommended by DroneKit
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            rp.loginfo(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        rp.loginfo('Armed')

        if aTargetAltitude > 0:
            rp.loginfo("Taking off!")

            thrust = DEFAULT_TAKEOFF_THRUST
            while True:
                current_altitude = self.vehicle.location.global_relative_frame.alt
                rp.loginfo(" Altitude: %f  Desired: %f" %
                           (current_altitude, aTargetAltitude))
                # Trigger just below target alt.
                if current_altitude >= aTargetAltitude*0.95:
                    rp.loginfo("Reached target altitude")
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

    '''
        Callbacks
    '''

    def joy_callback(self, data):
        # Empty Command
        self.cmds = Twist()

        # Joystick Controls
        self.cmds.linear.x = data.axes[2] * 10.0               # roll
        self.cmds.linear.y = data.axes[3] * 10.0               # pitch
        self.cmds.angular.z = data.axes[0] * 10.0               # yaw
        self.cmds.linear.z = 0.25 * (data.axes[1] / 2.0) + 0.5  # thrust

        # Button Controls
        self.arm = data.buttons[9]

        if data.buttons[4] == 1.0:
            if not self.engage_controller:
                rp.loginfo("Controller Engaged")
            else:
                rp.loginfo("Controller Disengaged")
            self.engage_controller = not self.engage_controller

    def clipCommand(self, cmd, upperBound, lowerBound):
        if cmd < lowerBound:
            cmd = lowerBound
        elif cmd > upperBound:
            cmd = upperBound

        return cmd

    '''
        Main Loop
    '''

    def send_commands(self):
        rp.loginfo("Accepting Commands")
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            # Arm motors
            if self.arm > 0:
                rp.loginfo("Arming...")
                self.arm_and_takeoff_nogps()

            if self.engage_controller:
                # Get latest transform
                try:
                    (T, R) = self.tf_listener.lookupTransform('UAV', 'bundle', rp.Time())
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                lastTFtime = self.tf_listener.getLatestCommonTime('raspicam', 'bundle')

                # Get current pose wrt Tag
                qx = R[0]
                qy = R[1]
                qz = R[2]
                qw = R[3]
                orientation = to_rpy(qw, qx, qy, qz)
                q_BwD = to_quaternion(roll  = -math.degrees(self.vehicle.attitude.roll),
                                      pitch = -math.degrees(self.vehicle.attitude.pitch),
                                      yaw   =  orientation[2])
                q_BwD_i = [q_BwD[0], -q_BwD[1], -q_BwD[2], -q_BwD[3]]
                t_BwD = [0, T[0], T[1], T[2]]

                t_DwB = quatMultiply(q_BwD_i, quatMultiply(t_BwD, q_BwD))

                # Drone position and orientation wrt bundle
                w_drone = orientation[2]
                x_drone = t_DwB[1]
                y_drone = t_DwB[2]
                z_drone = t_DwB[3]

                print("Drone position wrt the Bundle -> x = " + str(x_drone) + ", y = " + str(y_drone) + ", z = " + str(z_drone) + ", w = " + str(w_drone))

                # Set desired position
                des_position = [0, 0, -0.5]  # x, y, and z

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
                dw = w_drone

                # Translate error from optitrack frame to drone body frame
                drone_error = tag_to_drone(dx, dy, w_drone)

                x_error = drone_error[0]
                y_error = drone_error[1]
                z_error = dz

                if dw > 180.0:
                    w_error = dw - 360.0
                else:
                    w_error = dw

                # Update errors
                self.pid_w.updateError(w_error)
                self.pid_x.updateError(x_error)
                self.pid_y.updateError(y_error)
                self.pid_z.updateError(z_error)

                # Get commands
                dt_ros = rp.get_rostime() - lastTFtime
                dt = dt_ros.secs + dt_ros.nsecs * 1e-9

                if dt < 2.0:
                    angular_z_cmd = self.clipCommand(self.pid_w.getCommand(), 5, -5)
                    linear_z_cmd  = self.clipCommand(self.pid_z.getCommand() + 0.5, 0.65, 0.35)
                    linear_y_cmd  = self.clipCommand(self.pid_y.getCommand() - 1.3, 7.5, -7.5)
                    linear_x_cmd  = self.clipCommand(self.pid_x.getCommand() + 0.45, 7.5, -7.5)
                else:
                    angular_z_cmd = 0.0
                    linear_z_cmd  = 0.5
                    linear_y_cmd  = -1.3
                    linear_x_cmd  = 0.45
                    rp.logwarn("Marker lost for more than 2 seconds!!! Hovering")

                # Apply commands
                self.set_attitude(roll_angle=-linear_y_cmd,
                                  pitch_angle=-linear_x_cmd,
                                  yaw_angle=None,
                                  yaw_rate=angular_z_cmd, use_yaw_rate=True,
                                  thrust=linear_z_cmd,
                                  duration=1.0/self.rate)

                # Publish commands for logging
                cmd = TwistStamped()
                cmd.header.stamp = rp.Time.now()
                cmd.twist.angular.x = -linear_y_cmd
                cmd.twist.angular.y = -linear_x_cmd
                cmd.twist.angular.z = angular_z_cmd
                cmd.twist.linear.z  = linear_z_cmd
                self.command_pub.publish(cmd)
            r.sleep()

# Start Node
magdrone = magdroneControlNode()
