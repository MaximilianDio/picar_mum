#! /usr/bin/env python
import numpy as np
from picar.parameters import Picar, Wheel



class ControllerValues(object):
    """Object that holds desired/actual values for the controller."""

    def __init__(self, distance_x, distance_y):
        """

        Args:
            distance_x (float): Vehicle's distance to the center line of the track in x.
            distance_y (float): Vehicle's distance to the center line of the track in y.
            phi (float): Vehicle's angle relative to the center line of the track.
        """

        # TODO if integrator time is needed
        self.distance_x = distance_x
        self.distance_y = distance_y
        self.distance = np.sqrt(distance_x * distance_x + distance_y * distance_y)  # Additional Distance



class PathTrackingFF:

    def __init__(self):

        self.kappa = 0.0 # Curvature
        self.xLA = 1  # look a head error

        self.Kp = 1 # init Kp

        #Model based params
        self.mass = Picar.mass
        self.Inertia = Picar.inertia_z
        self.a = Picar.length - Picar.cog_x   # a is length front <- cog
        self.b = Picar.cog_x  # b is length rear <- cog COS in rear axis

        # cornering stiffness parameter
        self.C_f = Wheel.ksv  # N/rad
        self.C_r = Wheel.ksh  # N/rad

        # constants
        self.C0 = self.C_f + self.C_r
        self.C1 = self.a*self.C_f - self.b*self.C_r
        self.C2 = self.a**2*self.C_f - self.b**2*self.C_r




    def get_control_output(self):
        #discrete form of controller

    error=
    delta_phi=


    delta_fb = -Kp*error-Kp*self.xLA*delta_phi

    delta_ff =

    self.delta =  delta_ff + delta_fb

    def update_parameters(self, k_pvel=None, k_psteer=None,
                          k_dvel=None, k_dsteer=None):

        """Updates the controller picar.

        Args:
             k_pvel (float): The proportional gain of the controller.

            k_psteer (float): Proportional Gain steering

            k_dvel (float): D gain

            k_dsteer (float): D gain

        """

        if k_pvel is not None:
            self.K_p["vel"] = k_pvel

        if k_psteer is not None:
            self.K_p["steer"] = k_psteer

        if k_dvel is not None:
            self.K_d["vel"] = k_dvel

        if k_dsteer is not None:
            self.K_d["steer"] = k_dsteer

