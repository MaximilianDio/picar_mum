class VelocityPicker(object):

    def __init__(self, vel_reference, switch_bound):
        """
        Returns:
        """
        self.number_avg = 3  # number of stored values for radii estimation
        self.ave_list = [0.0] * self.number_avg  # array storing the last several values of the radii
        self.maximal_velocity = 2.5  # maximal velocity
        self.minimal_velocity = vel_reference  # minimal/basic velocity
        self.switch_bound = switch_bound  # radius at which size the car should switch to fast velocity
        self.last_cur_estimated_radius = 0

    def get_velocity(self, curve_point):

        ave = self.moving_average(curve_point)
        if ave > self.switch_bound:
            vel_out = self.maximal_velocity
        else:
            vel_out = self.minimal_velocity
        return vel_out

    def moving_average(self, curve_point):

        if curve_point.cR > 10 or curve_point.cR == float('nan'):
            curve_point.cR = 2.5

        # assign sign to radius  based on midpoint of circle
        # TODO test
        if curve_point.cy > 0:
            cur_estimated_radius = curve_point.cR
        else:
            cur_estimated_radius = - curve_point.cR

        self.ave_list.append(cur_estimated_radius)
        self.ave_list.pop(0)
        return sum(self.ave_list) / len(self.ave_list)

    def update_velocity_picker(self, maximal_velocity, minimal_velocity, switch_bound):
        """
        Args:
            maximal_velocity:       new maximal velocity the picker should choose on lane and radii bigger than switch_bound
            minimal_velocity:       new minimal velocity the picker should choose if the lane radii is smaller than switch_bound
            switch_bound:           Bound that is used to distinguish between minimal and maximal velocity
        Returns:
        """

        self.minimal_velocity = minimal_velocity
        self.maximal_velocity = maximal_velocity
        self.switch_bound = switch_bound
