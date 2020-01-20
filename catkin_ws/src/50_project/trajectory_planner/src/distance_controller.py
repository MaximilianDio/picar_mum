class PIDDistanceController:
    def __init__(self, Kp, Kd, Ki):
        self.__Kp = abs(Kp)
        self.__Kd = abs(Kd)
        self.__Ki = abs(Ki)

        self.__integrated_error_distance = 0.0
        self.old_time = 0

    def control(self, des_distance, cur_distance, rel_velocity, time):
        """ controls distance with PID controller
        @:returns control_command - can be used as des_speed command"""

        # calc delta time
        dt = time - self.old_time
        self.old_time = time

        # error distance if des_distcane - cur_distance negative increase speed -> minus sign
        errorDistance = -(des_distance - cur_distance)

        # integrate error
        self.__integrated_error_distance += errorDistance * dt

        # Control inputs
        # bring car to desired distance
        U_p = self.__Kp * errorDistance
        # keep velocity the same as obstacle
        U_d = self.__Kd * rel_velocity
        # control input on integrated error
        U_i = self.__Ki * self.__integrated_error_distance

        return U_d + U_p + U_i

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
