

class Controller(object):


    def __init__(self, Kp, Kd):
        """
        Args:
            Kp (list of proportional gains [Velocity, Steering])
            Kd (list of derivative gains [Velocity, Steering])
        """
        self.Kp = Kp
        self.Kd = Kd
        self.Radius = None
        self.CirclePos = CirclePos
        self.ObjectDetected = False
        self.CurrentVel = None
        self.DesiredVel = 0.0
        self.DesiredAngle = 0.0
        self.CurveClass = 0

    def CurveClassification(self, Rad, MidPoint):
        self.Radius = Rad
        self.CirclePos = MidPoint
        if self.Radius is inf:
            self.CurveClass = 1  # Straight Line
        elif self.CirclePos.y > 0:
            self.CurveClass = 2  # Right Turn
        elif self.CirclePos.y < 0:
            self.CurveClass = 3  # Left Turn
        else:
            self.CurveClass = 0  # Error

    



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

        # compute angle error
        error_angle = actual_values.angle - desired_values.angle

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
