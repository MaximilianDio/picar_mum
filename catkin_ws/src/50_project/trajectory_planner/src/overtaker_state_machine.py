class OvertakeStateMachine:

    def __init__(self, switch_params, min_dist_obstacle):
        # initial state
        self.current_state = "1"

        # have to be provided by node e.g. sensor data or calculations !
        self.switch_params = switch_params

        # updated by velocity estimator node
        self.own_velocity = 0.0

        self.time = 0.0  # initial time (starts when state machine gets first pacemaker message)
        self.overtake_start_time = 0.0

        self.curve_point = None  # CurvePoint2D
        self.rel_obstacle_point = None  # Point2D
        self.rel_obstacle_velocity = None  # Point2D

        self.min_dist_obstacle = min_dist_obstacle  # defined in yaml file
        self.overtake_time = None  # calculated during transition to state 3

        # calculated in transition to state 3 and 5
        self.t_trajectory = None

        # TODO init all controller types and values (as class objects)

        # return values for car command
        self.des_velocity = None
        self.des_angle = None

    def state_switcher(self):

        if self.current_state == "1":
            # call state 1
            self.state_1()

        elif self.current_state == "2":
            # call state 2
            self.state_2()

        elif self.current_state == "3*":
            # call state
            self.state_3_star()

        elif self.current_state == "3":
            # call state 3
            self.state_3()

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
        if self.curve_point is not None:
            # TODO: use curve position and derive necessary action e.g. speed and steering angle
            # implement controller run method
            # self.curve_point is CurvePoint2D
            pass

        # decide switch
        if self.switch_params["object_detection"]:
            self.current_state = "2"
            return
        # otherwise stay in state

    def state_2(self):
        # run state
        if self.curve_point is not None:
            # TODO: use curve position and derive necessary action e.g. speed and steering angle (same as in state 1)
            pass

        # change state if needed
        if not self.switch_params["object_detection"]:
            self.current_state = "1"
            return
        else:
            if self.rel_obstacle_point.x > self.min_dist_obstacle:
                # stay in state 2
                return
            else:
                if not self.switch_params["overtake"]:
                    self.current_state = "3*"
                    return
                else:
                    self.init_state_3_transition()
                    self.current_state = "3"

    def state_3_star(self):
        # run state
        if self.curve_point is not None and self.rel_obstacle_point is not None and self.rel_obstacle_velocity is not None:
            # TODO: object and line detection control, control speed to keep desired distance and angle to stay on curve
            # self.curve_point is CurvePoint2D and rel_obstacle_point and rel_obstacle_velocity is Point2D
            pass

        # change state if needed
        if not self.switch_params["object_detection"]:
            self.current_state = "1"
            return
        else:
            if not self.switch_params["overtake"]:
                # stay in state
                return
            else:
                self.init_state_3_transition()
                self.current_state = "3"
                return

    def init_state_3_transition(self):
        # reset overtaker tome (zero time)
        self.overtake_start_time = self.time

        # TODO calculate necessary overtake time (based on velocity and position of own vehicle and obstacle)
        self.overtake_time = None  # TODO: calculate here (state 3 and 4)
        self.t_trajectory = None  # TODO: calculate necessary time to fulfill open loop maneuver (only trajectory)

    def state_3(self):
        # run state

        # time spend in this state
        state_time = self.time - self.overtake_start_time
        print "time spent in state 3: " + state_time

        # TODO open loop control with overtaking time

        # change state if needed
        if state_time < self.t_trajectory:
            # stay in state
            return
        else:
            # FIXME do we need to reset time for state 4???
            if self.switch_params["line_detection"]:
                self.current_state = "4"
                return
            else:
                self.current_state = "4*"
                return

    def state_4(self):
        # run state

        # time spend in this state and state 3
        state_time = self.time - self.overtake_start_time
        print "time spent in state 4,4* and 3: " + state_time

        if self.curve_point is not None:
            # TODO: use curve position and derive necessary action e.g. speed and steering angle (offset required!!)
            # self.curve_point is CurvePoint2D
            pass

        # change state if needed
        if state_time < self.overtake_time:
            if not self.switch_params["line_detection"]:
                self.current_state = "4*"
                return
            # stay in state
            return
        else:
            self.init_state_5_transition()
            self.current_state = "5"
            return

    def state_4_star(self):
        # run state

        # time spend in this state and state 3
        state_time = self.time - self.overtake_start_time
        print "time spent in state 4,4* and 3: " + state_time

        # TODO control openloop

        # change state if needed
        if self.switch_params["cur_overtake_time"] < self.overtake_time:
            if self.switch_params["line_detection"]:
                self.current_state = "4"
                return
            # stay in state
            return
        else:
            self.init_state_5_transition()
            self.current_state = "5"
            return

    def init_state_5_transition(self):
        # reset overtaker time
        self.overtake_start_time = self.time

        self.t_trajectory = None  # TODO: calculate necessary time to fulfill open loop maneuver (only openloop trajectory)

    def state_5(self):
        # run state

        state_time = self.time - self.overtake_start_time
        print "time spent in state 5: " + state_time

        # TODO open loop control with overtaking time

        # TODO decide when to switch to 5*
        if state_time <= self.t_trajectory:
            # stay in state
            return
        else:
            self.current_state = "1"
            return

    def state_5_star(self):
        # TODO (necessary??!)
        pass
