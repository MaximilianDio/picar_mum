class VelocityPicker(object):

    def __init__(self, vel_mix, vel_max, switch_bound_slope, switch_bound_radius):
        """
        Returns:
        """
        self.number_avg = 2  # number of stored values for radii estimation
        self.ave_list = [[0.0] * self.number_avg,
                    [0.0] * self.number_avg]  # array storing the last several values of the radii
        self.maximal_velocity = vel_max  # maximal velocity
        self.minimal_velocity = vel_mix  # minimal/basic velocity
        # radius/slope at which size the car should switch to fast velocity
        self.switch_bound_slope = switch_bound_slope
        self.switch_bound_radius = switch_bound_radius
    def get_velocity(self, curve_point):

        ave = self.moving_average(curve_point)

        if abs(ave[1]) >= self.switch_bound_radius and abs(ave[0]) < self.switch_bound_slope:
            vel_out = self.maximal_velocity
        else:
            vel_out = self.minimal_velocity
        return vel_out

    def moving_average(self, curve_point):

        """

        Args:
            curve_point:

        Returns:
                list with [ave_slop,ave_radius]
        """

        self.ave_list[0].append(curve_point.slope)
        self.ave_list[0].pop(0)
        self.ave_list[1].append(curve_point.cR)
        self.ave_list[1].pop(0)

        return [sum(self.ave_list[0]) / len(self.ave_list[0]), sum(self.ave_list[1]) / len(self.ave_list[1])] #average slop and radius

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
