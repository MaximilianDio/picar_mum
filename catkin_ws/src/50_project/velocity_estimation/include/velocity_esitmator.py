class VelocityEstimator(object):
    """
    Converts Encoderdata to COM Velocity
    """

    def __init__(self, radius):
        self.radius_wheel = radius

    def getVelocity(self, rawdata):
        return rawdata * self.radius_wheel

    def getCOMvel(self, rawdata):
        velocity_est = self.getVelocity(rawdata)
        return float(velocity_est[0] + (velocity_est[1] - velocity_est[0]) / 2)
