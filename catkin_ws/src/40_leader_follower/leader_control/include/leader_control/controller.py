# Imports
import numpy as np

#class declaration

class ControllerValues(object):
    """Object that holds desired/actual values for the controller."""

    def __init__(self, distance_x, distance_y, phi):
        """

        Args:
            distance_x (float): Vehicle's distance to the center line of the track in x.
            distance_y (float): Vehicle's distance to the center line of the track in y.
            phi (float): Vehicle's angle relative to the center line of the track.
        """
        self.distance_x = distance_x
        self.distance_y = distance_y
        self.phi = phi
        self.distance = np.sqrt(distance_x * distance_x + distance_y * distance_y)  # Additional Distance


class Controller(object):
    """PD - Controller"""

    def __init__(self, k_pvel, k_psteer, k_dvel, k_dsteer):
        """

        Args:
            k_pvel (float): The proportional gain of the controller.
            
            k_psteer (float):
            
            k_dvel (float):
            
            k_dsteer (float):
            
        """

        self.K_p = dict(
            vel=k_pvel,
            steer=k_psteer,
        )

        self.K_d = dict(
            vel=k_dvel,
            steer=k_dsteer,
        )

    def get_control_output(self, desired_values, actual_values):
        """Calculates steering angle and velocity output based on desired and  actual values.

        Args:
            desired_values (ControllerValues): Desired values for distance and
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
        error_distance = actual_values.distance - desired_values.distance

        # compute velocity error
        error_velocity = actual_values.velocity - desired_values.velocity

        # compute angle error
        error_angle = actual_values.angle - desired_values.angle

        # compute angle error
        error_angularvelocity = actual_values.angularvelocity - desired_values.angularvelocity

        # calculate proportional output
        steering_angle_output = self.K_p["distance"] * error_combined

        ##################################################################################################
        ## hier aufgehoert
        #TODO  pd controller implementieren

        # calculate proportional output
        steering_angle_output = self.p_gain * error_combined

        # do not control velocity. just set it to desired value
        velocity_output = desired_values.velocity
        errors = (error_distance, error_angle, error_combined)
        return steering_angle_output, velocity_output, errors

        ##################################################################################################

    def update_parameters(self, k_pvel=None, k_psteer=None,
                          k_dvel=None, k_dsteer=None):
        """Updates the controller picar.

        Args:
            :param k_dvel:
            :param k_pvel:
            :param k_dsteer:
            :param k_psteer:

        """

        if k_pvel is not None:
            self.K_p["vel"] = k_pvel

        if k_psteer is not None:
            self.K_p["steer"] = k_psteer

        if k_dvel is not None:
            self.K_d["vel"] = k_dvel

        if k_dsteer is not None:
            self.K_d["steer"] = k_dsteer
