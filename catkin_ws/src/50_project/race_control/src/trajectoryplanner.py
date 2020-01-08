import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class OvertakingTrajectory():

    def __init__(self):
        traj = np.genfromtxt("V0to0.csv", delimiter=',')
        self.time = traj[0, :]
        self.velocity = traj[1, :]
        self.delta = traj[2, :]

    def get_feedforward_control(self, current_time):
        if current_time > self.time[len(self.time)-1]:
            return 0, 0
        else:
            velocity_interp = interp1d(self.time, self.velocity)
            delta_interp = interp1d(self.time, self.delta)
            return velocity_interp(current_time), delta_interp(current_time)

test = OvertakingTrajectory()

curTime = 1.4
curVel, curDelta = test.get_feedforward_control(curTime)

fig, ax = plt.subplots()
ax.plot(test.time, test.velocity)
ax.plot(test.time, test.delta)
ax.plot(curTime, curVel, marker='o')
ax.plot(curTime, curDelta, marker='o')
ax.grid()
plt.show()

