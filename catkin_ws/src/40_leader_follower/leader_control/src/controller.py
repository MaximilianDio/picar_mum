# Imports
import numpy as np
from picar import Picar


# class declaration
class ControllerValuesDesired(object):
    """Object that holds desired/actual values for the controller."""

    def __init__(self, distance, velocity):
        """

        Args:
            distance_ (float): Vehicle's distance to the center line of the track in x.
            veloctiy (float): Vehicle's veloctiy.
        """
        self.distance = distance
        self.velocity = velocity


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


class Controller(object):
    """PD - Controller"""

    def __init__(self, k_pvel, k_psteer, k_dvel, k_dsteer):
        """

        Args:
            k_pvel (float): The proportional gain of the controller.

            k_psteer (float): Proportional Gain steering

            k_dvel (float): D gain

            k_dsteer (float): D gain

        """

        self.K_p = dict(
            vel=k_pvel,
            steer=k_psteer,
        )

        self.K_d = dict(
            vel=k_dvel,
            steer=k_dsteer,
        )

        self.picar_fun = Picar()

    def get_control_output(self, desired_values, actual_values, last_values, time_diff):
        """Calculates steering angle and velocity output based on desired and  actual values.

        Args:
            desired_values (ControllerValuesDesired): Desired values for distance and
                angle offset relative to track. Used to compute control output.
            actual_values (ControllerValues): Actual values for distance and
                angle offset relative to track. Used to compute control output.

        Returns:
            steering_angle_output (float): The steering angle that should be
                sent to the car.
            velocity_output (float): The velocity that should be sent to the
                car.
            errors (tuple): Contains distance, angle and combined error.

        """


        # compute distance error
        error_distance = 30*actual_values.distance_x - desired_values.distance

        # compute displacement error in y direction
        error_y = actual_values.distance_y

        error_dy = 0.0
        error_velocity = 0.0

        # Control Design (Simple PI Controller)
        # todo resolve get_velocity and get angle error  - call method in the right way

        # Control Input Velocity
        if actual_values.distance == 0:
            velocity_output = 0.0
            steering_angle_output = 0.0
        else:
            # if time_diff == 0 or time_diff < 0:
                velocity_output = self.K_p["vel"] * error_distance  #self.picar_fun.get_velocity(self.K_p["vel"] * error_distance)
                steering_angle_output = self.picar_fun.get_angle(self.K_p["steer"] * error_y)
            # else:
            #     error_dy = (actual_values.distance_x - last_values.distance_x) / time_diff
            #     error_velocity = (actual_values.distance_x - last_values.distance_x) / time_diff
            #     velocity_output = self.picar_fun.get_velocity(self.K_p["vel"] * error_distance + self.K_d["vel"] * error_velocity)  # input meters per seconds output 0-1
            #     steering_angle_output = self.picar_fun.get_angle(
            #         self.K_p["steer"] * error_y + self.K_d["steer"] * error_dy)

        print(error_distance)
        errors = (error_distance, error_velocity, error_y, error_dy)

        return steering_angle_output, velocity_output, errors

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
