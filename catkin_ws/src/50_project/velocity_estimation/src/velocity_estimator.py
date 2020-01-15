import numpy as np
import rospy

class VelocityEstimator(object):
    """
    Converts Encoderdata to COM Velocity
    """

    def __init__(self, radius):
        self.radius_wheel = radius

        # Parameters for Kalman Filter
        self.Q = np.array([[0.05, 0.0], [0.0, 0.01]])
        self.R = 0.01  # 0.03

        self.x_hat = np.array([0, 0]).T
        self.P = np.zeros((2, 2))
        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)
        self.last_time = self.cur_time
        self.vel_com_last = 0.0

    def getVelocity(self, rawdata):
        return [x*self.radius_wheel for x in rawdata]

    def getCOMvel(self, rawdata, imu_data):
        # TODO mapacceleration correctly
        ax = 10 * imu_data.linear_acceleration.x  # Float64
        velocity_est = self.getVelocity(rawdata)
        velocity_com = float(velocity_est[0] + (velocity_est[1] - velocity_est[0]) / 2)

        dt = self.cur_time - self.last_time
        # Prediction step
        A = np.array([[1, -dt], [0, 1]])
        self.x_hat = np.dot(A, self.x_hat) + np.dot(np.array([1, 0]).T, ax)

        # New covariance matrix
        self.P = A * self.P * A.T + self.Q

        # measurement update
        y = velocity_com - self.x_hat[0]

        # Kalman Gain
        K = np.dot(self.P, np.array([1, 0]).T) / (np.dot(np.array([1, 0]), np.dot(self.P, np.array([1, 0]).T)) + self.R)
        self.x_hat = self.x_hat + K * y

        self.last_time = self.cur_time

        return self.x_hat[0]