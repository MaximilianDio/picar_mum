import numpy as np


class SpeedPlanner:

    # use function mapping by dictionary speed_profiles
    def constant_speed(self, time):
        return self.v1

    def linear_speed(self, time):
        # clip acceleration
        a = min(self.a_max, np.abs((self.v1 - self.v0) / self.T))
        # calculate sign of clipped acceleration
        a = a * np.sign((self.v1 - self.v0))
        return self.v0 + a * time

    def progressive_speed(self, time):
        # scenario 1 (a1 < a_max)
        a = (self.v1 - self.v0) / np.square(self.T)
        b = 0
        c = self.v0
        # scenario 2 (a1 = v'(T) = 2ab > a_max)
        if np.abs(2 * a * self.T) >= self.a_max:
            a_max = self.a_max * np.sign(self.v1 - self.v0)
            a = a_max / (2 * self.T)
            b = 0
            c = self.v0

        return a * np.square(time - b) + c

    def digressive_speed(self, time):
        # scenario 1 (a0 < a_max)
        a = (self.v0 - self.v1) / np.square(self.T)
        b = self.T
        c = self.v1
        # scenario 2 (a0 = v'(0) = -2ab > a_max)
        if np.abs(2 * a * b) >= self.a_max:
            a_max = self.a_max * np.sign(self.v1 - self.v0)
            a = - a_max / (2 * self.T)
            b = self.T
            c = self.v0 + a_max / 2 * self.T

        return a * np.square(time - b) + c

    speed_profiles = {
        "constant": constant_speed,
        "linear": linear_speed,
        "progressive": progressive_speed,
        "digressive": digressive_speed
    }

    def __init__(self, speed_profile, T, (v0, v1), a_max=0.5):
        self.set_max_acceleration(a_max)
        self.set_speed_profile(speed_profile)
        self.set_duration(T)
        self.set_speed((v0, v1))

    # max acceleration will be used to determine the acceleration at end or beginning of quadratic acceleration/
    # deceleration in m/s
    def set_max_acceleration(self, a_max):
        self.a_max = np.abs(float(a_max))

    # define speed profile from dictionary speed_profiles
    def set_speed_profile(self, speed_profile):
        if speed_profile in self.speed_profiles:
            self.speed_profile = speed_profile
        else:
            print "error: speed_profile" + speed_profile + "does not exist, it will be set to linear"
            self.speed_profile = "linear"

    # set duration to absolute time
    def set_duration(self, T):
        if T <= 0:
            print "error: duration can not be smaller than 0!, it will be set to 1 sec"
            T = 1

        self.T = float(T)

    # set speed interval for which the speed curve will be calculated
    def set_speed(self, (v0, v1)):
        self.v0 = float(v0)
        self.v1 = float(v1)

    # return speed
    def get_speed(self, t):
        # TODO what to do with out of bound time!
        # map function via dictionary
        speed_profile_func = self.speed_profiles[self.speed_profile]

        return speed_profile_func(self, t)
