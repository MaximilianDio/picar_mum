import numpy as np


class ControllerValues(object):
    """Object that holds desired/actual values for the controller."""
    def __init__(self, distance_x, distance_y, phi):
        """

        Args:
            distance (float): Vehicle's distance to the center line of the track.
            angle (float): Vehicle's angle relative to the center line of the track.
            velocity (float): Vehicle's velocity.
        """
        self.distance_x = distance_x
        self.distance_y = distance_y
        self.phi = phi
        self.distance =np.sqrt(distance_x*distance_x+distance_y*distance_y) #Additional Distance



class Controller(object):
    """PD - Controller"""
    def __init__(self, K_pvel, K_psteer, K_dvel, K_dsteer, weight_distance, weight_angle):
        """

        Args:
            p_gain (float): The proportional gain of the controller.
            weight_distance (float): This factor determines at which proportion
                the desired/actual distance influences the control output.
            weight_angle:  This factor determines at which proportion the
                desired/actual angle between track and vehicle influenes the
                control output.
        """

        self.K_p = dict(
            vel=K_pvel,
            steer=K_psteer,
        )

        self.K_d= dict(
            vel=K_dvel,
            steer=K_dsteer,
        )

        self.weights = dict(
            distance=weight_distance,
            angle=weight_angle,
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
        steering_angle_output = self.K_p["distance"] * error_combined ## hier aufgehört

##################################################################################################

        # compute the combined error
        error_combined = (error_distance * self.weights["distance"]
                          + error_angle * self.weights["angle"])

        # calculate proportional output
        steering_angle_output = self.p_gain * error_combined
        # do not control velocity. just set it to desired value
        velocity_output = desired_values.velocity
        errors = (error_distance, error_angle, error_combined)
        return steering_angle_output, velocity_output, errors

    def update_parameters(self,
                          p_gain=None,
                          weight_distance=None,
                          weight_angle=None):
        """Updates the controller picar.

        Args:
            p_gain (float): The proportional gain of the controller.
            weight_distance (float): This factor determines at which proportion
                the desired/actual distance influences the control output.
            weight_angle:  This factor determines at which proportion the
                desired/actual angle between track and vehicle influenes the
                control output.
        """
        if p_gain is not None:
            self.p_gain = p_gain
        if weight_distance is not None:
            self.weights["distance"] = weight_distance
        if weight_angle is not None:
            self.weights["angle"] = weight_angle
