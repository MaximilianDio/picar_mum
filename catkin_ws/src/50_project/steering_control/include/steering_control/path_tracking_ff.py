#! /usr/bin/env python
import numpy as np
from picar.parameters import Picar, Wheel


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

    def get_control_output(self, inputdata):
        """

        :param inputData: object of class ControlerValues
        :return: steering angle of vehicle
        """
        # discrete form of controller
        u_x = inputdata.u_x
        error = inputdata.error
        delta_psi = inputdata.delta_psi
        kappa = inputdata.kappa

        # feedback steering angle
        delta_fb = - self.Kp * (error + self.xLA * delta_psi)

        # feedforward steering angle due to curvature and longitudinal velocity
        g_ff = self.mass * u_x ** 2 / self.l * ((self.b * self.C_r + self.a * self.C_f * (self.Kp * self.xLA - 1)) / (
                self.C_r * self.C_f)) + self.l - self.b * self.Kp * self.xLA
        delta_ff = self.Kp_c * g_ff * kappa

        # total steering angle
        delta = delta_ff + delta_fb

        delta = self.picarfun.get_angle(delta / np.pi * 180)

        print('error: ' + str(error) + '\n')
        print('psi: ' + str(delta_psi) + '\n')
        print('delta_fb: ' + str(delta_fb / np.pi * 180) + '\n')
        print('delta_ff: ' + str(delta_ff / np.pi * 180) + '\n ------------------ \n')

        return delta

    def format_control_inputs(self, curve_point, velocity_estimated):
        # input for controller
        output_data = ControllerValues()

        output_data.error = - curve_point.y / curve_point.x * Picar.cog_x  # sign changed due to coor sys
        output_data.delta_psi = - np.arctan(curve_point.slope)  # sign changed due to coor sys
        try:
            if curve_point.cy > 0:  # assign sign to curvature based on midpoint of circle
                output_data.kappa = 1 / curve_point.cR
            else:
                output_data.kappa = - 1 / curve_point.cR
        except Exception:
            output_data.kappa = 0

        # data from encoders
        output_data.u_x = velocity_estimated

        return output_data

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


class ControllerValues(object):
    def __init__(self):

        self.error = 0.0  # lateral error
        self.delta_psi = 0.0  # relative orientation error

        self.u_x = 0.0  # current longitudinal velocity
        self.kappa = 0.0  # current curvature

    def set_values(self, error, delta_psi, u_x, kappa):

        if error is not None:
            self.error = error
        if delta_psi is not None:
            self.delta_psi = delta_psi
        if u_x is not None:
            self.u_x = u_x
        if kappa is not None:
            self.kappa = kappa
