#Imports
import numpy as np

class Wheel:
    def __init__(self):
        pass
    wheel_width=0.028
    wheel_diameter=0.065
    ksv= 46.04 #N/rad
    ksh=115.25 #N/rad


class  Picar(object):
    #All picar in SI Units

    def __init__(self):
        pass

    wheel=Wheel()
    track=0.162
    wheelbase=0.26
    ground_clearance=0.0012
    length=0.365
    width=0.190
    scale_to_real=0.1
    mass=2.323

    #Center of gravity
    cog_x=0.1182
    cog_y=0.0032
    cog_z=0.0279
    cog = np.array([cog_x, cog_y, cog_z])

    #Inertia of Mass
    inertia_z=0.0379


    ########Functions

    #velocity transform
    def get_velocity(self,x):

        # p1 = -0.76149
        # p2 = 2.6308
        # p3 = -0.3301
        #
        # y = p1 * x*x + p2 * x + p3tt
        p1 = 0.5824
        p2 = 0.1067

        if x > 0:
            y = p1 * x + p2
        else:
            y = p1 * x - p2

        return y

    #angle transform
    def get_angle(self,x):

        p1 = 0.87857
        p2 = 0.25

        y = p1*x + p2
        return y
