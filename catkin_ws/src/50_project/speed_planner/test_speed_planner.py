from src.speed_planner import VelocityPlanner
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    # try different speed_profiles: constant, linear, digressive, progressive
    speed_profile = VelocityPlanner("linear", 10, (0, 10), 1)

    Num = 100
    speed = np.zeros((Num, 1))
    time = np.linspace(-5.0, 15.0, num=Num)
    for i, t in enumerate(time):
        speed[i] = speed_profile.get_velocity(t)
    plt.plot(time, speed)
    plt.show()
