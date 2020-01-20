import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import rospkg
import os
from picar import Picar


# from discrete_integrator import *
import matplotlib.pyplot as plt

class OvertakingTrajectory(object):

    def __init__(self):
        file_path = os.path.join(rospkg.RosPack().get_path("race_control"),
                                 "src",
                                 "Overtake.csv")
        traj = np.genfromtxt(file_path, delimiter=',')
        self.time = traj[0, :]
        self.velocity = traj[1, :]
        self.delta = traj[2, :]
        self.helper_function = Picar()
        # self.integrator = DiscreteIntegrator(0.0, 0.0)
        self.cur_yaw = 0.0

    def get_feedforward_control(self, current_time):
        if current_time > self.time[len(self.time) - 1]:
            return 0.0, 0.0
        else:
            velocity_interp = interp1d(self.time, self.velocity)
            delta_interp = interp1d(self.time, self.delta)
            return velocity_interp(current_time), delta_interp(current_time) * 180 / np.pi  # Outputs: desVel, desAngle
