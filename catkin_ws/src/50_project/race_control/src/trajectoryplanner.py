import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import rospkg
import os
from picar import Picar

class OvertakingTrajectory(object):

    def __init__(self):
        file_path = os.path.join(rospkg.RosPack().get_path("race_control"),
                                            "src",
                                            "V0to0.csv")
        traj = np.genfromtxt(file_path, delimiter=',')
        self.time = traj[0, :]
        self.velocity = traj[1, :]
        self.delta = traj[2, :]
        self.helper_function = Picar()

    def get_feedforward_control(self, current_time):
        if current_time > self.time[len(self.time)-1]:
            return 0, 0
        else:
            velocity_interp = interp1d(self.time, self.velocity)
            delta_interp = interp1d(self.time, self.delta)
            return velocity_interp(current_time), -delta_interp(current_time)*180/np.pi  # Outputs: desVel, desAngle


test = OvertakingTrajectory()
#
curTime = 0.1
curVel, curDelta = test.get_feedforward_control(curTime)
#
fig, ax = plt.subplots()
ax.plot(test.time, test.velocity)
ax.plot(test.time, test.delta)
ax.plot(curTime, curVel, marker='o')
ax.plot(curTime, curDelta, marker='o')
ax.grid()
plt.show()

