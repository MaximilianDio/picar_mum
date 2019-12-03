# Imports
import numpy as np
import picar as picar

#class declaration

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

    def get_control_output(self, desired_values, actual_values, last_values):
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

        #space for integration
        last_values=last_values

        # compute distance error
        error_distance = actual_values.distance - desired_values.distance

        # compute velocity error
        error_velocity = 0 #actual_values.velocity - desired_values.velocity

        # compute displacement error in x direction
        error_y = actual_values.distance_y

        # compute velocity error in x
        error_dy = 0


        # Control Design (Simple PI Controller)

        # Control Input Velocity
        velocity_output = picar.get_velocity(self.K_p["vel"] * error_distance + self.K_d["vel"] * error_velocity)

        # Control Input Steering Angle
        steering_angle_output = picar.get_angle(self.K_p["steer"] * error_y + self.K_d["steer"] * error_dx)


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
