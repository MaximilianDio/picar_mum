class OvertakeStateMachine:

    def __init__(self, switch_params, min_dist_obstacle):
        # initial state
        self.current_state = "1"

        # have to be provided by node e.g. sensor data or calculations !
        self.switch_params = switch_params

        self.curve_point = None
        self.obstacle_point = None

        self.min_dist_obstacle = min_dist_obstacle
        self.dist_overtake = None  # TODO calculate online

        # TODO calculate necessary trajectory time
        self.t_trajectory = 100
        # TODO calculate necessary trajectory time
        self.t_trajectory_star = 100

    def state_switcher(self):

        if self.current_state == "1":
            # call state 1
            self.state_1()

        elif self.current_state == "2":
            # call state 2
            self.state_2()

        elif self.current_state == "3":
            # call state 3
            self.state_3()

        elif self.current_state == "3*":
            # call state
            self.state_3_star()

        elif self.current_state == "4":
            # call state
            self.state_4()

        elif self.current_state == "4*":
            # call state
            self.state_4_star()

        elif self.current_state == "5":
            # call state
            self.state_5()

        elif self.current_state == "5*":
            # call state
            self.state_5_star()

        else:
            raise ValueError

    def state_1(self):
        # run state
        # TODO

        # decide switch
        if self.switch_params["object_detection"]:
            self.current_state = "2"
            return
        # otherwise stay in state

    def state_2(self):
        # run state
        # TODO

        # change state if needed
        if not self.switch_params["object_detection"]:
            self.current_state = "1"
            return
        else:
            if self.switch_params["cur_dist_obstacle"] > self.min_dist_obstacle:
                # stay in state 2
                return
            else:
                if self.switch_params["overtake"]:
                    self.current_state = "3"
                    return
                else:
                    self.current_state = "3*"

    def state_3(self):
        # run state
        # TODO

        # change state if needed
        if self.switch_params["t"] < self.t_trajectory:
            # stay in state
            return
        else:
            if self.switch_params["line_detection"]:
                self.current_state = "4"
                return
            else:
                self.current_state = "4*"
                return

    def state_3_star(self):
        # run state
        # TODO

        # change state if needed
        if not self.switch_params["overtake"]:
            # stay in state
            return
        else:
            self.current_state = "3"
            return

    def state_4(self):
        # run state
        # TODO

        # change state if needed
        if self.switch_params["cur_dist_overtake"] < self.dist_overtake:
            if not self.switch_params["line_detection"]:
                self.current_state = "4*"
                return
            # stay in state
            return
        else:
            self.current_state = "5"
            return

    def state_4_star(self):
        # run state
        # TODO

        # change state if needed
        if self.switch_params["cur_dist_overtake"] < self.dist_overtake:
            if self.switch_params["line_detection"]:
                self.current_state = "4"
                return
            # stay in state
            return
        else:
            self.current_state = "5"
            return

    def state_5(self):

        # TODO decide when to switch to 5*
        if self.switch_params["t"] <= self.t_trajectory_star:
            # stay in state
            return
        else:
            self.current_state = "1"
            return

    def state_5_star(self):
        # TODO
        pass
