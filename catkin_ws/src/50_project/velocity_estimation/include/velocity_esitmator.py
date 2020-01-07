class VelocityEstimator(object):
    """
    Converts Encoderdata to COM Velocity
    """
    def __init__(self, params):
        self.rawdata = [0.0, 0.0]  # [left_angular, right_angular]
        self.velocity_est = [0.0, 0.0]  # [left_velocity, right_velocity]
        self.radius_wheel = 0.1  # TODO radius anpassen

    def getVelocity(self, rawdata):
        self.rawdata=rawdata
        return rawdata * self.radius_wheel

    def getCOMvel(self,rawdata):
        self.velocity_est = self.getVelocity(rawdata)
        return self.velocity_est[0]+(self.velocity_est[1]-self.velocity_est[0])/2
