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
    def __init__(self):
        # Number of states
        self.n = 12

        # System state
        self.X = np.matrix(np.zeros((self.n, 1)))

        # Initial State Transition Matrix
        self.F = np.asmatrix(np.eye(self.n))

        # Initial Process Matrix
        self.P = np.asmatrix(1.0e3 * np.eye(self.n))

        # Process Noise Level
        self.N = 1.0e-2

        # Initialize H and R matrices for optitrack pose
        self.Hopti = np.matrix(np.zeros((6, self.n)))
        self.Hopti[0:3, 0:3] = np.matrix(np.eye(3))
        self.Hopti[3:6, 6:9] = np.matrix(np.eye(3))

        self.Ropti = np.matrix(np.zeros((6, 6)))
        self.Ropti[0:3, 0:3] = 1.0e-3 * np.matrix(np.eye(3))
        self.Ropti[3:6, 3:6] = 1.0e-3 * np.matrix(np.eye(3))

        # Initialize Kalman Filter
        self.kalmanF = KalmanFilter()
        self.isInit = False
        self.lastTime = None

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

    def get_state(self):
        if self.isInit:
            timeNow = time.time()
            dt = timeNow - self.lastTime

            # Get a prediction
            self.kalmanF.updateF(dt)
            return self.kalmanF.get_prediction()
        else:
            return None
