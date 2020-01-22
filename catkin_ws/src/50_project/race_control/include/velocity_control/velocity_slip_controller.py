from std_msgs.msg import Float32
import rospy
from picar.parameters import Picar, Wheel
import numpy as np


class VelocitySlipController(object):

    def __init__(self, kp, kd):
        """
        Args:
            Kp (list of proportional gains [Velocity, Steering])
            Kd (list of derivative gains [Velocity, Steering])
        """
        self.kp = kp
        self.kd = kd
        self.Kp_slip = 1
        self.fudge = 0.95
        self.current_vel = 0.0
        self.current_slip_e = 0.0
        self.desired_vel = 0.0
        self.last_vel = self.current_vel
        self.last_slip_e = self.current_slip_e
        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)
        self.last_time = self.cur_time
        self.error = 0.0
        self.error_integral = 0.0
        self.sat = 2.0
        self.last_control = 0.0

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

        # contact parameters
        self.mu = 0.01  # friction coefficient

        self.picarfun = Picar()

    def get_velocity_output(self, curve_point, meas):

        self.current_vel = meas

        slip_error = self.fudge * meas ** 2 / curve_point.cR * self.mass - (
                    self.mu * np.e(-self.current_vel / self.vel_zero) * (
                        self.C_f * (delta - beta - self.a * yaw_dot / self.current_vel)
                        + self.C_r * (beta - self.b * yaw_dot / self.current_vel)))

        dt = self.cur_time - self.last_time

        dse = self.current_slip_e - self.last_slip_e

        self.desired_vel = self.Kp_slip * slip_error + self.Kd_slip * (dse / dt)



        if abs(self.desired_vel) < 0.2:
            self.last_control = 0.0

        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)
        dt = self.cur_time - self.last_time
        dv = self.current_vel - self.last_vel

        self.error = self.current_vel - self.desired_vel
        vel_output = -self.kp * self.error + self.last_control - self.kd * (dv / dt)
        self.last_control = vel_output
        if abs(vel_output) > self.sat:
            pass
        else:
            self.error_integral = self.error_integral + self.error * dt

        self.last_time = self.cur_time
        self.last_vel = self.current_vel
        self.last_vel = self.current_vel

        if vel_output < 0:
            print("Velocity negativ!!!")
            vel_output = 0

        return vel_output

    def update_gains(self, kp, kd):
        self.kp = kp
        self.kd = kd
