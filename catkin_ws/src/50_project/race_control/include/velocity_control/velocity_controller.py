import rospy


class VelocityController(object):

    def __init__(self, kp, kd):
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

        self.init_time = rospy.get_rostime()
        self.init_time = self.init_time.secs + float(self.init_time.nsecs * 1e-9)

        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)

        self.cur_time = self.cur_time - self.init_time

        self.last_time = self.cur_time
        self.error = 0.0
        self.error_integral = 0.0
        self.sat = 2.5
        self.last_control = 0.0

    def get_velocity_output(self, des_vel, meas):

        # ---------------- update attributes -------------------------
        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)

        self.cur_time = self.cur_time - self.init_time

        self.current_vel = meas
        self.desired_vel = des_vel

        # ---------------- assemble control_values -------------------------
        if abs(des_vel) < 0.2:
            self.last_control = 0.0

        dt = self.cur_time - self.last_time
        dv = self.current_vel - self.last_vel

        if dt == 0.0:
            dt = 1

        self.error = self.current_vel - self.desired_vel

        # ---------------- compute vel_cmd -------------------------
        vel_output = self.last_control + (- self.kp * self.error - self.kd * (dv / dt))

        self.last_control = vel_output
        self.last_time = self.cur_time
        self.last_vel = self.current_vel

        # ----------------Limit Vel CMD-------------------------

        if vel_output < 0:
            print("Velocity negativ!!!")
            vel_output = 0

        return vel_output

    def update_gains(self, kp, kd):
        self.kp = kp
        self.kd = kd
