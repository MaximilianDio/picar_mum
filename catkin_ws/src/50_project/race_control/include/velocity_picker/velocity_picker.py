
class VelocityPicker(object):

    def __init__(self):
        """
        Returns:
        """
        self.number_avg = 3  # number of stored values for radii estimation
        self.ave_list = [0.0] * self.number_avg  # array storing the last several values of the radii
        self.maximal_velocity = 2.5  # maximal velocity
        self.minimal_velocity = 0.8  # minimal/basic velocity
        self.switch_bound = 2  # radius at which size the car should switch to fast velocity

    def get_velocity(self, cur_estimated_radius):

        ave = self.moving_average(cur_estimated_radius)
        if ave > self.switch_bound:
            vel_out = self.maximal_velocity
        else:
            vel_out = self.minimal_velocity
        return vel_out

    def moving_average(self, new_val):
        if new_val > 10 or new_val == float('nan'):
            new_val = 2.5

        self.ave_list.append(new_val)
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
