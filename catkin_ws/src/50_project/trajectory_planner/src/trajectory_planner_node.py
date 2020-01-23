#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import Point32
from picar_msgs.srv import SetValue, SetBool, SetSteeringParametersList, SetDistanceParametersList
from std_msgs.msg import Float32
from picar_msgs.msg import CarCmd, MsgCurvePoint2D
from picar_common.picar_common import get_param, get_config_file_path, set_param
from overtaker_state_machine import OvertakeStateMachine
from picar_common.curve import *

# ros message for float values if nothing was detected
DEFAULT_FALSE_FLOAT_VALUE = float("inf")


class TrajectoryPlanner:

    def __init__(self):
        self._params = {}
        self.publishers = {}
        self.services = {}

        self.time = 0.0
        self.obstacle_old_pos = None
        self.obstacle_old_time = 0.0

        # import parameters from config yaml files
        config_file_name = get_param("~config_file_name", "default")

        config_file_path = get_config_file_path("trajection_planner",
                                                config_file_name)

        # shut down node if no config file could be found
        if config_file_path is None:
            rospy.signal_shutdown("Could not find any config file... "
                                  "Shutting down!")

        # read the config parameters form .yaml file
        self.setup_params(config_file_path)

        self.debug_flag = self._params["Debug"]
        self.apriltag_quatschwert = self._params["apriltag_quatschwert"]

        # register all publishers
        self.init_subscribers()

        # register all publishers
        self.init_publishers()

        # create all services
        self.init_services()

        # create state machine
        self.state_machine = OvertakeStateMachine(self._params)

    # ----------------------- initialization -----------------------------

    def setup_params(self, config_file_path):
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def init_services(self):
        self.services["overtake_command"] = rospy.Service(
            "~overtake_command",
            SetBool,
            self.service_overtake_clb)

        self.services["set_desired_distance"] = rospy.Service(
            "~set_desired_obstacle_distance",
            SetValue,
            self.service_des_obstacle_distance_clb)

        self.services["set_vel_reference"] = rospy.Service(
            "~set_vel_reference",
            SetValue,
            self.service_vel_reference_clb)

        self.services["set_switching_radius"] = rospy.Service(
            "~set_switching_radius",
            SetValue,
            self.service_switching_radius_clb)

        # ---------------- controller params -------------------------

        self.services["update_steering_parameters"] = rospy.Service(
            "~update_steering_parameters",
            SetSteeringParametersList,
            self.service_steering_params_clb)

        self.services["update_distance_controller_parameters"] = rospy.Service(
            "~update_distance_controller_parameters",
            SetDistanceParametersList,
            self.service_distance_params_clb)

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # relative position of leaders blue ball to picar
        self.publishers["des_car_command"] = rospy.Publisher("~des_car_cmd", CarCmd, queue_size=1)

    def init_subscribers(self):
        """ initialize ROS subscribers and stores them in a dictionary"""
        # pacemaker
        rospy.Subscriber("~pacemaker", Float32, self.run_node)
        # position of detected obstacle
        rospy.Subscriber("~obstacle_position", Point32, self.update_obstacle_pos_clb)
        # point on curve with position and circle
        rospy.Subscriber("~curve_point", MsgCurvePoint2D, self.update_curved_point_clb)
        # point on curve with position and circle
        rospy.Subscriber("~own_velocity", Float32, self.update_own_velocity_clb)

    # --------------------- Service Callbacks ----------------------------

    def service_overtake_clb(self, request):
        """Sets values via service"""
        self.state_machine.switch_params["overtake"] = request.value

        return 1

    def service_des_obstacle_distance_clb(self, request):
        """Sets values via service"""
        self.state_machine.des_distance_to_obstacle = request.value

        return 1

    def service_vel_reference_clb(self, request):
        """Sets values via service"""
        self.state_machine.velocity_control_12.minimal_velocity = request.value
        return 1

    def service_switching_radius_clb(self, request):
        """ set switching radius of velocity picker"""
        self.state_machine.velocity_control_12.switch_bound = request.value
        return 1

    def service_steering_params_clb(self, request):
        self.state_machine.steering_control_123star.Kp = request.Kp
        self.state_machine.steering_control_123star.Kp_c = request.Kp_c
        self.state_machine.steering_control_123star.xLA = request.xLA

        return 1

    def service_distance_params_clb(self, request):
        self.state_machine.velocity_control_3star.set_Kp(request.Kp)
        self.state_machine.velocity_control_3star.set_Kd(request.Kd)
        self.state_machine.velocity_control_3star.set_Ki(request.Ki)

        return 1

    # ---------------------- Topic Callbacks -----------------------------

    def update_obstacle_pos_clb(self, message):
        quatschwert = self.apriltag_quatschwert

        obstacle = Point2D([message.x, message.y])

        # update relative position of obstacle
        if obstacle.x == DEFAULT_FALSE_FLOAT_VALUE or obstacle.x < quatschwert \
                or obstacle.y == DEFAULT_FALSE_FLOAT_VALUE:
            self.state_machine.switch_params["object_detection"] = False
            self.state_machine.rel_obstacle_point = None
            self.state_machine.rel_obstacle_velocity = None
        else:
            self.state_machine.switch_params["object_detection"] = True
            self.state_machine.rel_obstacle_point = obstacle
            # update relative velocity of obstacle
            self.state_machine.rel_obstacle_velocity = self.estimate_rel_obstacle_velocity(obstacle)

    def update_curved_point_clb(self, message):
        # message of type MsgCurvePoint2D
        curve_point = message
        # update curvepoint
        if curve_point.x == DEFAULT_FALSE_FLOAT_VALUE or curve_point.x == DEFAULT_FALSE_FLOAT_VALUE:
            self.state_machine.switch_params["line_detection"] = False
            self.state_machine.curve_point = None
        else:
            self.state_machine.switch_params["line_detection"] = True
            self.state_machine.curve_point = curve_point

    def update_own_velocity_clb(self, message):
        self.state_machine.own_velocity_est = message.data

    def run_node(self, time):
        """ main callback which will be called by pacemaker node, updates all necessary data to state machine and runs
        it state machine calls corresponding trajectory planner which updates desired velocity and angle!"""

        time = rospy.get_rostime()
        self.time = time.secs + time.nsecs*1e-9

        # update time in state machine
        self.state_machine.time = self.time

        if self.debug_flag == True:
            print "-----------------------------------------------------------------------------------"
            print "-----------------------------------------------------------------------------------"
            print "---------------------" + str(self.state_machine.time) + "--------------------------"
            print "current state: " + str(self.state_machine.current_state)
            print "curve can be detected: " + str(self.state_machine.switch_params["line_detection"])
            if self.state_machine.switch_params["line_detection"]:
                print "curve point: x: " + str(self.state_machine.curve_point.x) + " y: " + str(
                    self.state_machine.curve_point.y)
            print "obstacle can be detected: " + str(self.state_machine.switch_params["object_detection"])
            print "own velocity: " + str(self.state_machine.own_velocity_est)
            if self.state_machine.switch_params["object_detection"]:
                print "obstacle at position: x: " + str(self.state_machine.rel_obstacle_point.x) + " y: " + str(
                    self.state_machine.rel_obstacle_point.y)
                print "min distance to obstacle: " + str(self.state_machine.min_dist_obstacle)
                print "too close: " + str(self.state_machine.rel_obstacle_point.x < self.state_machine.min_dist_obstacle)
                if self.state_machine.rel_obstacle_velocity is not None:
                    print "relative obstacle velocity: x: " + str(
                        self.state_machine.rel_obstacle_velocity.x) + " y: " + str(
                        self.state_machine.rel_obstacle_velocity.y)
            print "can overtake: " + str(self.state_machine.switch_params["overtake"])

        # run states and switch states if needed
        self.state_machine.state_switcher()

        # get desired message and publish!
        des_car_command = CarCmd()

        # TODO decide what to do when none is returned - IDEA return 0.0
        des_car_command.velocity = self.state_machine.des_velocity
        des_car_command.angle = self.state_machine.des_angle

        self.publishers["des_car_command"].publish(des_car_command)

    def estimate_rel_obstacle_velocity(self, point):

        velocity = None
        # use old and new position (and time) to calculate velocity
        if self.obstacle_old_pos is not None and self.time != 0.0:
            # calculate differences
            delta_x = point.x - self.obstacle_old_pos.x
            delta_y = point.y - self.obstacle_old_pos.y
            delta_t = self.time - self.obstacle_old_time

            # differences quotient
            if delta_t != 0.0:
                velocity = Point2D([delta_x / delta_t, delta_y / delta_t])

        # update points
        self.obstacle_old_pos = point

        return velocity


if __name__ == "__main__":
    rospy.init_node("trajectory_planner")
    TrajectoryPlanner()
    rospy.spin()
