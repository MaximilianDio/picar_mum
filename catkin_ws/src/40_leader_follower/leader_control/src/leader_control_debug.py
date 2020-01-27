#!/usr/bin/env python
from src.controller import Controller, ControllerValues, ControllerValuesDesired
from src.picar import Picar

picar = Picar()

a = Picar.get_angle(picar, 10.0)

desired_values = ControllerValuesDesired(0.2,
                                          0.0)

actual_values = ControllerValues(0.3,0.3)

K_p = dict(
vel=3.0,
steer=1.0,
    )

K_d = dict(
    vel=1.0,
    steer=0.1,
)

def get_control_output(desired_values, actual_values,K_p ,K_d):
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
    error_distance = actual_values.distance - desired_values.distance

    # compute velocity error
    error_velocity = 0 #actual_values.velocity - desired_values.velocity

    # compute displacement error in y direction
    error_y = actual_values.distance_y

    # compute velocity error in x
    error_dy = 0 # actual_values.dy


    # Control Design (Simple PI Controller)
    #todo resolve get_velocity and get angle error

    picar = Picar()
    # Control Input Velocity
    velocity_output = Picar.get_velocity(picar,K_p["vel"] * error_distance + K_d["vel"] * error_velocity) #input meters per seconds output 0-1

    # Control Input Steering Angle
    steering_angle_output =Picar.get_angle(picar,K_p["steer"] * error_y + K_d["steer"] * error_dy) #input degree -output virtual degree


    errors = (error_distance, error_velocity, error_y, error_dy)

    return steering_angle_output, velocity_output, errors




PD = Controller(3.0,1.0,1.0,0.1)


(steering_angle, velocity, errors) = PD.get_control_output(desired_values, actual_values, last_values=None)




print (steering_angle)
print(velocity)