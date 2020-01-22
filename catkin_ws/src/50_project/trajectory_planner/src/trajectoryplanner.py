import numpy as np
from scipy.interpolate import interp1d
import rospkg
import os


class OvertakingTrajectory(object):

    def __init__(self):
        file_path = os.path.join(rospkg.RosPack().get_path("trajectory_planner"),
                                 "src",
                                 "Overtaker_moving_object.csv")
        traj = np.genfromtxt(file_path, delimiter=',')
        self.time = traj[0, :]
        self.velocity = traj[1, :]
        self.angle = traj[2, :]

    def get_feedforward_control(self, current_time):
        if current_time > self.time[len(self.time) - 1]:
            return 0.0, 0.0
        else:
            velocity_interp = interp1d(self.time, self.velocity)
            delta_interp = interp1d(self.time, self.angle)
            return velocity_interp(current_time), delta_interp(current_time) * 180 / np.pi  # Outputs: desVel, desAngle
