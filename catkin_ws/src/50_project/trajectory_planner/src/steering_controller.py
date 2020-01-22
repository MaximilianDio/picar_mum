#! /usr/bin/env python
import numpy as np
from picar.parameters import Picar, Wheel
from picar_common.curve import CurvePoint2D


class PathTrackingFF(object):

    def __init__(self, Kp, Kp_c, xLA):

        self.xLA = xLA  # look a head error
        self.Kp = Kp  # init Kp
        self.Kp_c = Kp_c  # init Kp

        # Model based params
        self.mass = Picar.mass
        self.Inertia = Picar.inertia_z
        self.a = Picar.length - Picar.cog_x  # a is length front <- cog
        self.b = Picar.cog_x  # b is length rear <- cog COS in rear axis
        self.l = Picar.length

        # cornering stiffness parameter
        self.C_f = Wheel.ksv  # N/rad
        self.C_r = Wheel.ksh  # N/rad

        # constants
        self.C0 = self.C_f + self.C_r
        self.C1 = self.a * self.C_f - self.b * self.C_r
        self.C2 = self.a ** 2 * self.C_f - self.b ** 2 * self.C_r

        self.picarfun = Picar()

    def get_steering_output(self, curve_point, velocity_estimated):

        if curve_point.slope == float("inf"):
            delta_psi = 0.0  # only use psi if 3 points are detected - look a head term less influence !
        else:
            delta_psi = - np.arctan(curve_point.slope)  # sign changed due to coor sys

        # feedback steering angle

        if curve_point.x == float("inf"):
            print("No useable Imagedata")
            error = 0
            delta_psi = 0.0
        else:
            error = - curve_point.y / curve_point.x * Picar.cog_x  # sign changed due to coor sys

        # assign sign to curvature based on midpoint of circle
        try:
            if curve_point.cy > 0:  # assign sign to curvature based on midpoint of circle
                kappa = 1 / curve_point.cR
            else:
                kappa = - 1 / curve_point.cR
        except Exception:
            kappa = 0

        # data from encoders
        u_x = velocity_estimated

        # feedback steering angle
        delta_fb = - self.Kp * (error + self.xLA * delta_psi)

        # feedforward steering angle due to curvature and longitudinal velocity
        g_ff = self.mass * u_x ** 2 / self.l * ((self.b * self.C_r + self.a * self.C_f * (self.Kp * self.xLA - 1)) / (
                self.C_r * self.C_f)) + self.l - self.b * self.Kp * self.xLA

        delta_ff = self.Kp_c * g_ff * kappa  # only computed if 3 points are avail

        # total steering angle
        delta = delta_ff + delta_fb

        delta = self.picarfun.get_angle(delta / np.pi * 180)

        return delta

    def update_parameters(self, Kp, Kp_c, xLA):

        """
        :param xLa: float32 lookahead distance
        :param Kp: float32 P gain of Controller
        :return:
        """
        if xLA is not None:
            self.xLA = xLA

        if Kp is not None:
            self.Kp = Kp

        if Kp_c is not None:
            self.Kp_c = Kp_c
