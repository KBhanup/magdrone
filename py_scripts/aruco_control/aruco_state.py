#!/usr/bin/python

import sys

import numpy as np
import time

import rospy as rp
import tf

from geometry_msgs.msg import PoseStamped, TwistStamped

from kalman_filter import KalmanFilter

from quaternion import Quaternion
from quaternion import quat2rpy
from quaternion import rpy2quat

import threading


class FilterNode():
    def __init__(self,):
        rp.init_node("filter")

        # Number of states
        self.n = 12

        # System state
        self.X = np.matrix(np.zeros((self.n, 1)))

        # Initial State Transition Matrix
        self.F = np.asmatrix(np.eye(self.n))

        # Initial Process Matrix
        self.P = np.asmatrix(1.0e3 * np.eye(self.n))

        # Process Noise Level
        self.N = 1.0e-3

        # Initialize H and R matrices for optitrack pose
        self.Hopti = np.matrix(np.zeros((6, self.n)))
        self.Hopti[0:3, 0:3] = np.matrix(np.eye(3))
        self.Hopti[3:6, 6:9] = np.matrix(np.eye(3))

        self.Ropti = np.matrix(np.zeros((6, 6)))
        self.Ropti[0:3, 0:3] = 1.0 * 1.0e-2 * np.matrix(np.eye(3))
        self.Ropti[3:6, 3:6] = 1.5 * 1.0e-1 * np.matrix(np.eye(3))

        # Initialize Kalman Filter
        self.kalmanF = KalmanFilter()
        self.isInit = False
        self.lastTime = None

        # Set up Publisher
        self.state_pub = rp.Publisher(
            "aruco_state/pose", PoseStamped, queue_size=1)
        self.state_rate_pub = rp.Publisher(
            "aruco_state/rates", TwistStamped, queue_size=1)

        # Create thread for publisher
        self.rate = 30
        t = threading.Thread(target=self.statePublisher)
        t.start()

        rp.spin()

    def state_update(self, T, R, mTime):
        attiQ = Quaternion(R[0], R[1], R[2], R[3])
        rpy = quat2rpy(attiQ)

        pose = np.matrix([[T[0]],
                          [T[1]],
                          [T[2]],
                          [rpy[0]],
                          [rpy[1]],
                          [rpy[2]]])

        if self.isInit:
            dt = mTime - self.lastTime

            # Prediction
            self.kalmanF.updateF(dt)
            self.kalmanF.updateQ()
            self.kalmanF.predict()

            # Correction
            self.kalmanF.correct(pose, self.Hopti, self.Ropti)

            # Update state
            self.X = self.kalmanF.getState()

            self.lastTime = mTime

        else:
            # Initialize state
            self.X[0] = pose[0]
            self.X[1] = pose[1]
            self.X[2] = pose[2]
            self.X[6] = pose[3]
            self.X[7] = pose[4]
            self.X[8] = pose[5]

            # Initialize filter
            self.kalmanF.initialize(self.X, self.F, self.P, self.N)
            self.lastTime = mTime
            self.isInit = True

    def statePublisher(self):
        r = rp.Rate(self.rate)

        while not rp.is_shutdown():
            if self.isInit:
                timeNow = time.time()
                dt = timeNow - self.lastTime
                self.lastTime = timeNow

                # Prediction
                self.kalmanF.updateF(dt)
                self.kalmanF.updateQ()
                self.kalmanF.predict()

                stateMsg = PoseStamped()
                stateMsg.header.stamp = timeNow
                stateMsg.header.frame_id = 'raspicam'

                stateMsg.pose.position.x = self.X.item(0)
                stateMsg.pose.position.y = self.X.item(1)
                stateMsg.pose.position.z = self.X.item(2)

                q = rpy2quat(self.X.item(6), self.X.item(7), self.X.item(8))

                stateMsg.pose.orientation.x = q.x
                stateMsg.pose.orientation.y = q.y
                stateMsg.pose.orientation.z = q.z
                stateMsg.pose.orientation.w = q.w

                twistMsg = TwistStamped()
                twistMsg.header.stamp = timeNow
                twistMsg.header.frame_id = 'raspicam'

                twistMsg.twist.linear.x = self.X.item(3)
                twistMsg.twist.linear.y = self.X.item(4)
                twistMsg.twist.linear.z = self.X.item(5)

                twistMsg.twist.angular.x = self.X.item(9)
                twistMsg.twist.angular.y = self.X.item(10)
                twistMsg.twist.angular.z = self.X.item(11)

                self.state_pub.publish(stateMsg)
                self.state_rate_pub.publish(twistMsg)

            r.sleep()
