from distance_controller import PIDDistanceController
from steering_controller import PathTrackingFF
from velocity_picker import VelocityPicker
from trajectoryplanner import OvertakingTrajectory
from picar.parameters import Picar


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
        self.vel_reference = params_dict["vel_reference"]  # desired velocity for state 1,2

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
        # --------------------------------------------------------------/home/mattivahs/picar_mum/catkin_ws/src/50_project/trajectory_planner/src
        # -- steering controller for state 1, 2 and 3*
        # --------------------------------------------------------------
        Kp_steering = params_dict["Kp_steering"]
        Kp_c_steering = params_dict["Kp_c_steering"]
        xLA_steering = params_dict["xLA_steering"]
        self.steering_control_123star = PathTrackingFF(Kp_steering, Kp_c_steering, xLA_steering)

        # -- velocity controller for state 1,2
        # ----------------------------------------------
        self.velocity_control_12 = VelocityPicker()

        # -- PID DISTANCE controller for state 3*
        # ----------------------------------------------
        self.des_distance_to_obstacle = params_dict["des_distance_to_obstacle"]
        Kp_distance = params_dict["Kp_distance"]
        Kd_distance = params_dict["Kd_distance"]
        Ki_distance = params_dict["Ki_distance"]
        self.velocity_control_3star = PIDDistanceController(Kp_distance, Kd_distance, Ki_distance)

        # -- TRAJECTORY PLANNER FOR OVERTAKING
        # TODO MATTI: init class overting trajectory
        self.trajectory = OvertakingTrajectory()
        self.mappings = Picar()

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
            des_velocity = self.velocity_control_12.get_des_vel(self.curve_point.cR)
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
            des_velocity = self.velocity_control_12.get_des_vel(self.curve_point.cR)
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
                if not self.switch_params["overtake"] and self.rel_obstacle_point.x < 2 * self.min_dist_obstacle:
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

        # desired initial values
        self.des_velocity = self.trajectory.velocity[0]
        self.des_angle = self.trajectory.angle[0]

        # use overtake_start_time to calculate delta time (e.g. time spent in state)
        self.overtake_start_time = self.time

        # necessary overtake time (based on velocity and position of own vehicle and obstacle)
        self.t_trajectory = self.trajectory.time[len(self.trajectory.time) - 1]

    def state_3(self):
        """ open loop controlled overtaking maneuver, to other line"""

        # time spend in this state
        state_time = self.time - self.overtake_start_time
        print "time spent in state 3: " + str(state_time)

        # run controller
        # open loop control with overtaking time
        open_loop_velocity, open_loop_angle = self.trajectory.get_feedforward_control(state_time)

        # update desired angle and velocity according to mappings
        self.des_velocity = self.mappings.get_velocity(open_loop_velocity)
        self.des_angle = self.mappings.get_angle(open_loop_angle)

        # change state if needed
        if state_time < self.t_trajectory:
            # stay in state
            return
        else:
            self.current_state = "1"
            return