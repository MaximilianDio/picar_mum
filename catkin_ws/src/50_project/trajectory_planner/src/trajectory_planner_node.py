#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from picar_msgs.srv import SetValue
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

        # register all publishers
        self.init_subscribers()

        # register all publishers
        self.init_publishers()

        # create all services
        self.init_services()

        self.switch_params = {"line_detection": False,  # line can be detected
                              "object_detection": False,  # object detected
                              "overtake": False  # overtaking is allowed
                              }

        # create state machine
        self.state_machine = OvertakeStateMachine(self.switch_params,
                                                  self._params["min_dist_obstacle"])  # TODO input controller parameters

    # --------------------------------------------------------------------
    # ----------------------- initialization -----------------------------
    # --------------------------------------------------------------------

    def setup_params(self, config_file_path):
        with open(config_file_path, "r") as file_handle:
            data = yaml.safe_load(file_handle)
        self._params = data
        for param_name in self._params:
            set_param("~" + param_name, self._params[param_name])

    def init_services(self):
        pass

    def init_publishers(self):
        """ initialize ROS publishers and stores them in a dictionary"""
        # relative position of leaders blue ball to picar
        self.publishers["des_car_command"] = rospy.Publisher("~desired_car_command", CarCmd,
                                                             queue_size=1)

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

    def update_obstacle_pos_clb(self, message):
        obstacle = Point2D([message.x, message.y])

        # update relative position of obstacle
        if obstacle.x == DEFAULT_FALSE_FLOAT_VALUE or obstacle.y == DEFAULT_FALSE_FLOAT_VALUE:
            self.switch_params["object_detection"] = False
            self.state_machine.rel_obstacle_point = None
        else:
            self.switch_params["object_detection"] = True
            self.state_machine.rel_obstacle_point = obstacle

        # update relative velocity of obstacle
        self.state_machine.rel_obstacle_velocity = self.estimate_rel_obstacle_velocity(obstacle)

    def update_curved_point_clb(self, message):
        # message of type MsgCurvePoint2D
        curve_point = CurvePoint2D([message.x, message.y])
        curve_point.slope = message.slope
        curve_point.circle = Circle2D(message.cR, Point2D([message.x, message.y]))

        # update curvepoint
        if curve_point.x == DEFAULT_FALSE_FLOAT_VALUE or curve_point.x == DEFAULT_FALSE_FLOAT_VALUE:
            self.switch_params["line_detection"] = False
            self.state_machine.curve_point = None
        else:
            self.switch_params["line_detection"] = True
            self.state_machine.curve_point = curve_point

    def update_own_velocity_clb(self, message):
        self.state_machine.own_velocity = message.data

    def run_node(self, time):
        """ main callback which will be called by pacemaker node, updates all necessary data to state machine and runs
        it state machine calls corresponding trajectory planner which updates desired velocity and angle!"""

        self.time = time.data

        # update time in state machine
        self.state_machine.time = self.time

        # update values of switching parameters
        self.switch_params["overtake"] = True  # TODO use service!!!

        # update switching parameters in state machine
        self.state_machine.switch_params = self.switch_params

        # DEBUG can be deleted
        print "---------------------" + str(self.state_machine.time) + "--------------------------"
        print "current state: " + str(self.state_machine.current_state)
        print "curve can be detected: " + str(self.state_machine.switch_params["line_detection"])
        if self.state_machine.switch_params["line_detection"]:
            print "curve point: x: " + str(self.state_machine.curve_point.x) + " y: " + str(
                self.state_machine.curve_point.y)
        print "obstacle detected: " + str(self.state_machine.switch_params["object_detection"])
        print "own velocity: " + str(self.state_machine.own_velocity)
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

        # FIXME check if velocity is right, seems wrong!
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
