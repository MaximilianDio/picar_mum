
class VelocityController(object):

    def __init__(self, kp, kd=0.0):
        """
        Args:
            Kp (list of proportional gains [Velocity, Steering])
            Kd (list of derivative gains [Velocity, Steering])
        """
        self.kp = kp
        self.kd = kd
        self.current_vel = 0.0
        self.desired_vel = 0.0
        self.last_vel = self.current_vel

    def get_velocity_output(self, meas, des_vel):
        self.current_vel = meas
        self.desired_vel = des_vel
        vel_output = self.kp * (self.current_vel - self.desired_vel)
        self.last_vel = self.current_vel
        return vel_output

    def update_gains(self, kp, kd):
        self.kp = kp
        self.kd = kd
