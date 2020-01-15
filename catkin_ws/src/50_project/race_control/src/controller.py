from curve import *
from race_control.speed_planner import import *
from picar.parameters import Picar


class Controller(object):

    def __init__(self, kp, kd):
        """
        Args:
            Kp (list of proportional gains [Velocity, Steering])
            Kd (list of derivative gains [Velocity, Steering])
        """
        self.kp = kp
        self.kd = kd
        self.circle = None
        self.objectDetected = False
        self.currentVel = None
        self.desiredVel = 0.0
        self.desiredAngle = 0.0
        self.curveClass = 0
        self.velocities = [2.0, 1.0, 0.5]  # velocities for [straight line, r = 1, r = 0.75]
        self.lastCurveClass = 0
        self.startTimeAcc = 0.0  # start time for acceleration maneuver
        self.accTime = 0.5  # total time for acc maneuver
        self.changeCurveClass = False

    def curveClassification(self, circle):
        if isinstance(circle, Circle2D):
            self.circle = circle
        else:
            raise ValueError("Circle has to be a Circle2D object")

        if self.circle.radius is inf:
            self.curveClass = 1  # Straight Line
        elif self.circle.center.y > 0:
            self.curveClass = 2  # Right Turn
        elif self.circle.center < 0:
            self.curveClass = 3  # Left Turn
        else:
            self.curveClass = 0  # Error

    def updateGains(self, kp, kd):
        if isinstance(kp, list) and isinstance(kd, list):
            self.kp = kp
            self.kd = kd
        else:
            raise ValueError("Gains have to be specified in a list")

    def getControlOutputs(self, curVel, x, y, circle, curTime):
        self.curveClassification(circle)

        if self.curveClass == 1:
            self.desiredVel = self.velocities[0]
            angle_out = 0.0
        elif self.curveClass == 2:
            self.desiredVel = self.velocities[1]
            angle_out = Picar.length / circle.radius  # feedforward ackerman angle
        elif self.curveClass == 3:
            self.desiredVel = self.velocities[2]
            angle_out = - Picar.length / circle.radius  # feedforward ackerman angle
        else:
            self.desiredVel = 0.0
            angle_out = 0.0

        if self.curveClass is not self.lastCurveClass:
            self.startTimeAcc = curTime
            self.changeCurveClass = True
            speedPlanner = VelocityPlanner("linear", 0.5, (curVel, self.desiredVel))

        if self.changeCurveClass and (curTime - self.startTimeAcc) < self.accTime:
            self.desiredVel = speedPlanner.get_velocity(curTime - self.startTimeAcc)

        self.currentVel = curVel
        error_vel = curVel - self.desiredVel
        error_y = y

        velocity_out = self.kp[0] * error_vel
        angle_out = angle_out + self.kp[1] * error_y

        return velocity_out, angle_out
