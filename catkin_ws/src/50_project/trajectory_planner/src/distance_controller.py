class PIDDistanceController:
    def __init__(self, Kp, Kd, Ki):
        self.__Kp = abs(Kp)
        self.__Kd = abs(Kd)
        self.__Ki = abs(Ki)

        self.__integrated_error_distance = 0.0

    def control(self, des_distance, cur_distance):
        """ controls distance with PID controller
        @:returns control_command - can be used as des speed command"""

        errorDistance = des_distance - cur_distance
        self.integrated_error_distance += errorDistance
        # TODO implement derivative part
        control_command = self.Kp * errorDistance + self.Ki * self.integrated_error_distance

        return control_command

    def set_Kp(self, Kp):
        """ set Kp only with positive values"""
        if Kp > 0:
            self.__Kp = Kp

    def set_Kd(self, Kd):
        """ set Kp only with positive values"""
        if Kd > 0:
            self.__Kd = Kd

    def set_Ki(self, Ki):
        """ set Kp only with positive values"""
        if Ki > 0:
            self.__Ki = Ki

    def reset_integrated_error(self):
        self.__integrated_error_distance = 0.0
