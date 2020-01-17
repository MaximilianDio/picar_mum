import numpy as np
import rospy

class VelocityEstimator(object):
    """
    Converts Encoderdata to COM Velocity
    """

    def __init__(self, radius):
        self.radius_wheel = radius

        # Parameters for Kalman Filter
        self.Q = np.array([[0.001, 0.0], [0.0, 0.003]])
        self.R = 0.03  # 0.03

        self.x_hat = np.array([0, 0]).T
        self.P = np.eye(2)
        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)
        self.last_time = self.cur_time
        self.vel_com_last = 0.0
        self.H = np.array([1, 0])

    def getVelocity(self, rawdata):
        return [x*self.radius_wheel for x in rawdata]

    def prediction_step(self, imu_data):
        # TODO mapacceleration correctly
        ax = 9.81 * imu_data.linear_acceleration.x  # Float64

        self.cur_time = rospy.get_rostime()
        self.cur_time = self.cur_time.secs + float(self.cur_time.nsecs * 1e-9)

        dt = self.cur_time - self.last_time
        # Prediction step
        A = np.array([[1, -dt], [0, 1]])
        self.x_hat = np.dot(A, self.x_hat) + np.dot(np.array([dt, 0]), ax)

        # New covariance matrix
        # self.P = A * self.P * A.T + self.Q
        self.P = np.matmul(np.matmul(A, self.P), A.T) + self.Q
        self.last_time = self.cur_time
        print("bias: " + str(self.x_hat[1]))

        return self.x_hat[0]

    def correction_step(self, rawdata):
        velocity_est = self.getVelocity(rawdata)
        velocity_com = float(velocity_est[0] + (velocity_est[1] - velocity_est[0]) / 2)

        # measurement update
        y = velocity_com - self.x_hat[0]

        # Kalman Gain
        kalman_gain = np.array([self.P[0, 0], self.P[1, 0]]) * (1/(self.P[0, 0] + self.R))
        self.x_hat = self.x_hat + kalman_gain * y

        self.P = np.matmul((np.eye(2) - np.outer(kalman_gain, np.array([1, 0]))), self.P)

        return self.x_hat[0]
