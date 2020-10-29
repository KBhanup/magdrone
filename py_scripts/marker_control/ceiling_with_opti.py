#!/usr/bin/python

import rospy as rp
import threading
import math
import time

from marker_state import FilterNode

from dronekit import connect, VehicleMode
from pymavlink import mavutil

from std_msgs.msg import Int8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, TwistStamped


def to_rpy(qw, qx, qy, qz):
    """
    Convert quaternions to rpy
    """
    r = math.atan2(2 * (qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))

    sinP = 2 * (qw*qy - qz*qx)
    if abs(sinP) > 1:
        p = math.copysign(1.0, sinP) * math.pi / 2
    else:
        p = math.asin(sinP)

    y = math.atan2(2 * (qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    return [r, p, y]


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert rpy to quaternions
    """
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def quat_multiply(p, q):
    w = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3]
    x = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2]
    y = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1]
    z = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]

    return [w, x, y, z]


def rotate_vector(x, y, angle):
    x_r = x * math.cos(angle) - y * math.sin(angle)
    y_r = x * math.sin(angle) + y * math.cos(angle)
    return [x_r, y_r]


class magdroneControlNode():

    def __init__(self):
        rp.init_node("magdrone_deploy")

        # Connect to the Vehicle
        rp.loginfo('Connecting to Vehicle')
        self.vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=57600)

        # Create Filter Node
        self.filter = FilterNode()

        # Set up Controllers
        self.kp_z = 0.45
        self.kd_z = 0.3
        self.kp_y = 0.18
        self.kd_y = 0.15
        self.kp_x = 0.18
        self.kd_x = 0.15
        self.kp_w = 0.15

        self.z_error = 0.0
        self.y_error = 0.0
        self.x_error = 0.0
        self.w_error = 0.0
        self.z_error_d = 0.0
        self.y_error_d = 0.0
        self.x_error_d = 0.0

        # Variables
        self.lastOnline = 0
        self.arm = 0
        self.on_mission = False
        self.mission_id = 1
        self.state_id = 0
        self.magnet_engaged = True
        self.docked = True
        self.w_drone = 0.0
        self.target_x =  0.0
        self.target_y =  0.0
        self.deployed_x = self.target_x
        self.deployed_y = self.target_y

        # Desired positions for misions 1 and 2
        # Offset is reduced because of the addition of the sensor package
        self.struct_x = 0.0
        self.struct_y = 0.0
        self.struct_z = 2.00
        # Mission 1.
        self.desired_positions_m1 = [1.5, 1.4, 1.2, 1.0, 0.8, 0.6, 0.4, 0.15, 0.6]
        # Mission 2
        self.desired_positions_m2 = [0.4, 0.15, 0.4, 0.7, 1.0, 1.4]

        # Constant transformations
        self.q_CwD = [0, -0.7071, 0.7071, 0]
        self.q_CwD_i = [0, 0.7071, -0.7071, 0]
        self.t_CwD = [0, 0.165, 0, 0]

        # Set up Subscribers
        self.stag_sub = rp.Subscriber(
            "/stag_ros/bundles", PoseStamped, self.stag_callback, queue_size=1)
        self.joy_sub = rp.Subscriber(
            "/joy", Joy, self.joy_callback, queue_size=1)
        self.pose_sub = rp.Subscriber(
            "/opti_state/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.pose_sub = rp.Subscriber(
            "/opti_state/rates", TwistStamped, self.rate_callback, queue_size=1)

        # Set up Publishers
        self.setpoint_pub = rp.Publisher(
            "/setpoint", Int8, queue_size=1)
        self.command_pub = rp.Publisher(
            "/commands", TwistStamped, queue_size=1)
        self.state_pub = rp.Publisher(
            "/aruco_state/pose", PoseStamped, queue_size=1)
        self.state_rate_pub = rp.Publisher(
            "/aruco_state/rates", TwistStamped, queue_size=1)
        
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
        #while not self.vehicle.is_armable:
        #    rp.loginfo(" Waiting for vehicle to initialise...")
        #    time.sleep(1)

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
            yaw_rate,  # Body yaw rate in radian/second
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
        start = time.time()

        self.send_attitude_target(roll_angle, pitch_angle,
                                  yaw_angle, yaw_rate, use_yaw_rate,
                                  thrust)

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
        self.yaw_des = 90.0 # yaw in degrees

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
        drone_error = rotate_vector(self.x_error,self.y_error,self.yaw_position)

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
        drone_error_rate = rotate_vector(self.x_error_d,self.y_error_d,self.yaw_position)

        # Update x and y error rate
        self.x_error_d = drone_error_rate[0]
        self.y_error_d = drone_error_rate[1]

    def stag_callback(self, data):
        q_BwC = [data.pose.orientation.w,
                 data.pose.orientation.x,
                 data.pose.orientation.y,
                 data.pose.orientation.z]

        t_BwC = [0,
                 data.pose.position.x,
                 data.pose.position.y,
                 data.pose.position.z]

        q_BwD_pre = quat_multiply(self.q_CwD, q_BwC)
        t_BwD = quat_multiply(self.q_CwD, quat_multiply(
            t_BwC, self.q_CwD_i)) + self.t_CwD

        orientation = to_rpy(q_BwD_pre[0],
                             q_BwD_pre[1],
                             q_BwD_pre[2],
                             q_BwD_pre[3])

        roll = self.vehicle.attitude.roll
        pitch = self.vehicle.attitude.pitch

        tag_angles = rotate_vector(roll,
                                   pitch,
                                   -orientation[2])

        q_BwD = to_quaternion(roll=-tag_angles[0],
                              pitch=-tag_angles[1],
                              yaw=orientation[2])
        q_BwD_i = [q_BwD[0], -q_BwD[1], -q_BwD[2], -q_BwD[3]]

        t_DwB = quat_multiply(q_BwD_i, quat_multiply(t_BwD, q_BwD))

        T = [-t_DwB[1], -t_DwB[2], -t_DwB[3]]
        R = [-q_BwD[1], -q_BwD[2], -q_BwD[3], q_BwD[0]]

        self.lastOnline = time.time()
        self.filter.state_update(T, R, self.lastOnline)

    def joy_callback(self, data):
        # Button Controls
        self.arm = data.buttons[9]

        if data.buttons[0] == 1.0:
            rp.loginfo("Mission set to 1 - Deploying Sensor")
            self.mission_id = 1
            self.state_id = 0
        if data.buttons[1] == 1.0:
            rp.loginfo("Mission set to 2 - Retrieving Sensor and Exiting")
            self.mission_id = 2
            self.state_id = 0
        if data.buttons[2] == 1.0:
            rp.loginfo("Mission set to 3 - Emergency Escape and Hover")
            self.mission_id = 3
            self.state_id = 0
        if data.buttons[3] == 1.0:
            rp.loginfo("Mission set to 4 - Changing Flight Mode to Landing")
            self.mission_id = 4
            self.state_id = 0
        if data.buttons[4] == 1.0:
            if not self.on_mission:
                rp.loginfo("Starting Mission %d", self.mission_id)
            else:
                rp.loginfo("Mission Canceled")
            self.on_mission = not self.on_mission

    '''
        State Machine Functions
    '''

    def stateMachine(self, x_drone, y_drone, z_drone):
        if self.mission_id == 1:
            x_des = self.target_x
            y_des = self.target_y
            z_des = self.desired_positions_m1[self.state_id]

            check = self.checkState([x_drone, y_drone, z_drone], [x_des, y_des, z_des])

            if (self.state_id == len(self.desired_positions_m1) - 2):
                if not self.docked:
                    self.state_id += 1
                    rp.loginfo("New target altitude is %f", self.desired_positions_m1[self.state_id])
            else:
                if (check[0] < 0.05) & (check[1] < 0.05) & (check[2] < 0.075):
                    if (self.state_id < len(self.desired_positions_m1) - 1):
                        self.state_id += 1
                        rp.loginfo("New target altitude is %f", self.desired_positions_m1[self.state_id])

            z_des = self.desired_positions_m1[self.state_id]

        elif self.mission_id == 2:
            x_des = self.deployed_x
            y_des = self.deployed_y
            z_des = self.desired_positions_m2[self.state_id]

            check = self.checkState([x_drone, y_drone, z_drone], [x_des, y_des, z_des])

            if (self.state_id == 1):
                if self.docked:
                    self.state_id += 1
                    rp.loginfo("New target altitude is %f", self.desired_positions_m2[self.state_id])
            else:
                if (check[0] < 0.035) & (check[1] < 0.035) & (check[2] < 0.05):
                    if (self.state_id < len(self.desired_positions_m2) - 1):
                        self.state_id += 1
                        rp.loginfo("New target altitude is %f", self.desired_positions_m2[self.state_id])

            z_des = self.desired_positions_m2[self.state_id]

        elif self.mission_id == 3:
            x_des = 0.0
            y_des = 0.0
            z_des = 1.0

        elif self.mission_id == 4:
            rp.loginfo("Setting LAND mode")
            self.vehicle.mode = VehicleMode("LAND")
            x_des = x_drone
            y_des = y_drone
            z_des = z_drone

        return [x_des, y_des, z_des]

    def checkState(self, current_p, desired_p):
        dx = abs(current_p[0] - desired_p[0])
        dy = abs(current_p[1] - desired_p[1])
        dz = abs(current_p[2] - desired_p[2])

        return [dx, dy, dz]

    def clip_command(self, cmd, upperBound, lowerBound):
        if cmd < lowerBound:
            cmd = lowerBound
        elif cmd > upperBound:
            cmd = upperBound

        return cmd

    '''
        Magnet Control
    '''

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

        # Send command
        self.vehicle.send_mavlink(msg_hi)
        rp.loginfo("Magnet Engaged")
        time.sleep(5)
        self.vehicle.send_mavlink(msg_neut)
        rp.loginfo("Magnet in Neutral")
        self.docked = True

    def disengage_magnet(self):
        msg_low = self.vehicle.message_factory.command_long_encode(
                0, 0,   # target_system, target_command
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
                0,
                8,    # servo number
                982,  # servo position
                0, 0, 0, 0, 0)

        msg_neut = self.vehicle.message_factory.command_long_encode(
                0, 0,   # target_system, target_command
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # command
                0,
                8,    # servo number
                1500, # servo position
                0, 0, 0, 0, 0)

        # Send command
        self.vehicle.send_mavlink(msg_low)
        rp.loginfo("Magnet Disengaged")
        time.sleep(5)
        self.vehicle.send_mavlink(msg_neut)
        rp.loginfo("Magnet in Neutral")
        self.docked = False

        deployed_at = rotate_vector(self.x_error, self.y_error, -self.w_drone)
        self.deployed_x = deployed_at[0]
        self.deployed_y = deployed_at[1]
        self.target_x = self.deployed_x
        self.target_y = self.deployed_y
        rp.loginfo("Sensor deployed at (%f, %f)", self.deployed_x, self.deployed_y)

    def update_error(self, X):
        # Get current pose wrt Aruco
        self.w_drone = -X.item(8)
        x_drone = X.item(0)
        y_drone = X.item(1)
        z_drone = X.item(2)

        # Desired position
        des_position = self.stateMachine(x_drone, y_drone, z_drone)

        # Calculate error
        dx = des_position[0] - x_drone
        dy = des_position[1] - y_drone
        dz = z_drone - des_position[2]
        dw = self.w_drone

        # Translate error from optitrack frame to drone body frame
        drone_error = rotate_vector(dx, dy, self.w_drone)

        self.x_error = drone_error[0]
        self.y_error = drone_error[1]
        self.z_error = dz

        if dw > math.pi:
            self.w_error = dw - 2.0 * math.pi
        else:
            self.w_error = dw

        # Translate error rate from optitrack frame to drone body frame
        drone_error_rate = rotate_vector(-X.item(3), -X.item(4), self.w_drone)

        # Update rate errors
        self.x_error_d = drone_error_rate[0]
        self.y_error_d = drone_error_rate[1]
        self.z_error_d = X.item(5)

    def publish_state(self, X):
        timeNow = rp.Time.now()
        stateMsg = PoseStamped()
        stateMsg.header.stamp = timeNow
        stateMsg.header.frame_id = 'bundle'

        stateMsg.pose.position.x = X.item(0)
        stateMsg.pose.position.y = X.item(1)
        stateMsg.pose.position.z = X.item(2)

        q = to_quaternion(roll=X.item(6),
                          pitch=X.item(7),
                          yaw=X.item(8))

        stateMsg.pose.orientation.x = q[1]
        stateMsg.pose.orientation.y = q[2]
        stateMsg.pose.orientation.z = q[3]
        stateMsg.pose.orientation.w = q[0]

        twistMsg = TwistStamped()
        twistMsg.header.stamp = timeNow
        twistMsg.header.frame_id = 'bundle'

        twistMsg.twist.linear.x = X.item(3)
        twistMsg.twist.linear.y = X.item(4)
        twistMsg.twist.linear.z = X.item(5)

        twistMsg.twist.angular.x = X.item(9)
        twistMsg.twist.angular.y = X.item(10)
        twistMsg.twist.angular.z = X.item(11)

        self.state_pub.publish(stateMsg)
        self.state_rate_pub.publish(twistMsg)

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

            # Get current state and publish it
            if (time.time() - self.lastOnline < 1.0):
                X = self.filter.get_state()
                if X is not None:
                    # self.update_error(X)
                    t = threading.Thread(target=self.publish_state, args=[X])
                    t.start()

            # Mission has started
            if self.on_mission:
                if (time.time() - self.lastOnline < 1.0):
                    # Generate commands
                    uZ = self.kp_z * self.z_error + self.kd_z * self.z_error_d
                    uY = self.kp_y * self.y_error + self.kd_y * self.y_error_d
                    uX = self.kp_x * self.x_error + self.kd_x * self.x_error_d
                    uW = self.kp_w * self.w_error
                    # 7.5 degrees -> 0.131 rad
                    # 5.0 degrees -> 0.087 rad
                    # 1.3 degrees -> 0.023 rad
                    # 0.5 degrees -> 0.008 rad
                    linear_z_cmd = self.clip_command(uZ + 0.5, 0.65, 0.35)
                    linear_y_cmd = self.clip_command(uY - 0.011, 0.131, -0.131)
                    linear_x_cmd = self.clip_command(uX + 0.017, 0.131, -0.131)
                    angular_z_cmd = self.clip_command(uW, 0.087, -0.087)
                else:
                    linear_z_cmd = 0.5
                    linear_y_cmd = 0.0
                    linear_x_cmd = 0.0
                    angular_z_cmd = 0.0
                    rp.logwarn(
                        "Marker lost for more than a second!!! Hovering")

                # Apply commands
                self.set_attitude(roll_angle=-linear_y_cmd,
                                  pitch_angle=-linear_x_cmd,
                                  yaw_angle=None,
                                  yaw_rate=angular_z_cmd,
                                  use_yaw_rate=True,
                                  thrust=linear_z_cmd,
                                  duration=1.0/self.rate)

                # Publish commands for logging
                cmd = TwistStamped()
                cmd.header.stamp = rp.Time.now()
                cmd.twist.angular.x = linear_y_cmd
                cmd.twist.angular.y = -linear_x_cmd
                cmd.twist.angular.z = angular_z_cmd
                cmd.twist.linear.z = linear_z_cmd
                self.command_pub.publish(cmd)

                if (self.mission_id == 1) & (self.state_id == len(self.desired_positions_m1) - 2) & (self.magnet_engaged):
                    self.magnet_engaged = False
                    t = threading.Thread(target=self.disengage_magnet)
                    t.start()

                if (self.mission_id == 2) & (self.state_id == 1) & (not self.magnet_engaged):
                    self.magnet_engaged = True
                    t = threading.Thread(target=self.engage_magnet)
                    t.start()

            r.sleep()


# Start Node
magdrone = magdroneControlNode()
