from distance_controller import PIDDistanceController
from steering_controller import PathTrackingFF
from velocity_picker import VelocityPicker
import numpy as np


class OvertakeStateMachine:

    def __init__(self, params_dict):
        # initial state
        self.current_state = "1"

        self.switch_params = {"line_detection": False,  # line can be detected
                              "object_detection": False,  # object detected
                              "overtake": False  # overtaking is allowed
                              }

        # updated by velocity estimator node
        self.own_velocity_est = 0.0

        self.time = 0.0  # initial time (starts when state machine gets first pacemaker message)
        self.overtake_start_time = 0.0

        self.curve_point = None  # CurvePointMessage
        self.rel_obstacle_point = None  # Point2D
        self.rel_obstacle_velocity = None  # Point2D

        self.min_dist_obstacle = params_dict["min_dist_obstacle"]  # defined in yaml file
        self.overtake_time = None  # calculated during transition to state 3

        # calculated in transition to state 3 and 5
        self.t_trajectory = None

        # TODO init all controller types and values (as class objects)
        # initialize controllers
        # --------------------------------------------------------------
        # -- steering controller for state 1, 2 and 3*
        # --------------------------------------------------------------
        Kp_steering = params_dict["Kp_steering"]
        Kp_c_steering = params_dict["Kp_c_steering"]
        xLA_steering = params_dict["xLA_steering"]
        self.steering_control_123star = PathTrackingFF(Kp_steering, Kp_c_steering, xLA_steering)

        # -- velocity controller for state 1,2
        # ----------------------------------------------
        self.velocity_control_12 = VelocityPicker()
        self.velocity_control_12.minimal_velocity = params_dict["vel_reference"]  # desired velocity for state 1,2
        self.velocity_control_12.switch_bound = params_dict["switch_bound"]  # switching radius

        # -- PID DISTANCE controller for state 3*
        # ----------------------------------------------
        self.des_distance_to_obstacle = params_dict["des_distance_to_obstacle"]
        Kp_distance = params_dict["Kp_distance"]
        Kd_distance = params_dict["Kd_distance"]
        Ki_distance = params_dict["Ki_distance"]
        self.velocity_control_3star = PIDDistanceController(Kp_distance, Kd_distance, Ki_distance)

        # -- TRAJECTORY PLANNER FOR OVERTAKING
        # TODO MATTI: init class overting trajectory

        # return values for car command
        self.des_velocity = 0.0
        self.des_angle = 0.0

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

        else:
            raise ValueError

    def state_1(self):
        """ no obstacle detected - line controlled """
        # run controller

        if self.curve_point is not None:
            des_angle = self.steering_control_123star.get_steering_output(self.curve_point, self.own_velocity_est)
            des_velocity = self.velocity_control_12.get_velocity(self.curve_point.cR*np.sign(self.curve_point.cy))
        else:
            # stop car when no line is detected!
            des_angle = 0.0
            des_velocity = 0.0

        self.des_velocity = des_velocity
        self.des_angle = des_angle

        # decide switch
        if self.switch_params["object_detection"]:
            self.current_state = "2"
            return
        # otherwise stay in state

    def state_2(self):
        """ obstacle was detected but it is far awy - only line controlled"""
        # run controller
        if self.curve_point is not None:
            des_angle = self.steering_control_123star.get_steering_output(self.curve_point, self.own_velocity_est)
            des_velocity = self.velocity_control_12.get_velocity(self.curve_point.cR*np.sign(self.curve_point.cy))
        else:
            # stop car when no line is detected!
            des_angle = 0.0
            des_velocity = 0.0

        self.des_velocity = des_velocity
        self.des_angle = des_angle

        # change state if needed
        if not self.switch_params["object_detection"]:
            # go back to state 1 when vehicle can not be detected
            self.current_state = "1"
            return
        else:

            if self.rel_obstacle_point.x > self.min_dist_obstacle:
                # stay in state 2
                return
            else:
                if not self.switch_params["overtake"]:
                    self.current_state = "3*"
                    self.velocity_control_3star.reset_integrated_error()
                    return
                else:
                    self.init_state_3_transition()
                    self.current_state = "3"

    def state_3_star(self):
        """ keep distance to obstacle (drive with same velocity at a given distance) - see leader control"""
        # run controller

        # run controller
        if self.curve_point is not None:
            des_angle = self.steering_control_123star.get_steering_output(self.curve_point, self.own_velocity_est)
        else:
            des_angle = 0.0

        if self.rel_obstacle_point is not None and self.rel_obstacle_velocity is not None:
            des_velocity = self.velocity_control_3star.control(self.des_distance_to_obstacle, self.rel_obstacle_point.x,
                                                               self.rel_obstacle_velocity.x, self.time)
        else:
            # stop car when no line is detected!
            des_velocity = 0.0

        self.des_velocity = des_velocity
        self.des_angle = des_angle

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
        """ initialize necessary information before going to state 3 - e.g. calculate necessary overtake time,
        trajectory time ... """

        # TODO MATTI: change
        self.des_velocity = 0.8
        self.des_angle = 0.0

        # use overtake_start_time to calculate delta time (e.g. time spent in state)
        self.overtake_start_time = self.time

        # TODO MATTI: calculate necessary overtake time (based on velocity and position of own vehicle and obstacle)
        self.t_trajectory = 5.0  # TODO: calculate necessary time to fulfill open loop maneuver (only trajectory)

    def state_3(self):
        """ open loop controlled overtaking maneuver, to other line"""

        # time spend in this state
        state_time = self.time - self.overtake_start_time
        print "time spent in state 3: " + str(state_time)

        # run controller
        # TODO MATTI: open loop control with overtaking time

        # TODO: MATTI: -update desired angle and velocity
        self.des_velocity = 0.0
        self.des_angle = 0.0

        # change state if needed
        if state_time < self.t_trajectory:
            # stay in state
            return
        else:
            self.current_state = "1"
            return
            # FIXME do we need to reset time for state 4???
            # if self.switch_params["line_detection"]:
            #     self.current_state = "4"
            #     return
            # else:
            #     self.current_state = "4*"
            #     return
