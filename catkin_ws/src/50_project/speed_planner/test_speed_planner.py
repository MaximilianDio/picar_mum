from src.speed_planner import SpeedPlanner
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # try different speed_profiles: constant, linear, digressive, progressive
    speed_profile = SpeedPlanner("progressive", 10, (0, -10), 3)
    speed = np.zeros((21, 1))
    time = np.linspace(0.0, 10.0, num=21)
    for i, t in enumerate(time):
        speed[i] = speed_profile.get_speed(t)
    plt.plot(time, speed)
    plt.show()
