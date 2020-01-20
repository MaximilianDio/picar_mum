from std_msgs.msg import Float32
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
        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)
        self.last_time = self.cur_time
        self.error = 0.0
        self.last_control = 0.0

    def get_velocity_output(self, meas, des_vel):
        self.current_vel = meas
        self.desired_vel = des_vel

        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)
        dt = self.cur_time - self.last_time
        if dt == 0:
            dt = 1

        dv = self.current_vel - self.last_vel

        self.error = self.current_vel - self.desired_vel
        vel_output = self.last_control + (- self.kp * self.error - self.kd * (dv/dt))
        print("desired_vel: " + str(self.desired_vel))
        print("actual_vel: " + str(self.current_vel))
        self.last_control = vel_output

        self.last_time = self.cur_time
        self.last_vel = self.current_vel

        return vel_output
