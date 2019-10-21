#!/usr/bin/python

import os
import time
import pygame
import rospy
from picar_msgs.msg import CarCmd


class KeyboardController(object):
    """Class to control the vehicle by publishing ROS messages based on keyboard input.

    """
    K_LEFT = 0
    K_RIGHT = 1
    K_UP = 2
    K_DOWN = 3
    K_0 = 4
    K_1 = 5
    K_2 = 6
    K_3 = 7
    K_4 = 8
    K_5 = 9
    K_6 = 10
    K_7 = 11
    K_8 = 12
    K_9 = 13
    SCREEN_SIZE = 640

    def __init__(self):
        pygame.init()

        vehicle_name = rospy.get_namespace().strip("/")

        self.velocity = 0.3

        self.screen = pygame.display.set_mode(
            (KeyboardController.SCREEN_SIZE, KeyboardController.SCREEN_SIZE)
        )
        pygame.display.set_caption("Keyboard Control of {}"
                                   "".format(vehicle_name))
        # font = pygame.font.Font(None, 24)

        file_path = os.path.dirname(__file__)
        file_path = os.path.join(file_path, "..", "images")
        icon = pygame.image.load(file_path + "/icon.png")
        pygame.display.set_icon(icon)

        self.buttons = pygame.image.load(file_path + "/d-pad.png")
        self.buttons = pygame.transform.scale(
            self.buttons,
            (KeyboardController.SCREEN_SIZE, KeyboardController.SCREEN_SIZE)
        )

        button_pressed = pygame.image.load(file_path + "/d-pad-pressed.png")
        button_pressed = pygame.transform.scale(
            button_pressed,
            (KeyboardController.SCREEN_SIZE, KeyboardController.SCREEN_SIZE)
        )

        self.button_image = dict(
            forward=button_pressed,
            left=pygame.transform.rotate(button_pressed, 90),
            right=pygame.transform.rotate(button_pressed, 270),
            backward=pygame.transform.rotate(button_pressed, 180),
        )

        # send directly to motor_node. Due to gazebo remapping won't work as
        # expected.
        # see:
        # https://answers.ros.org/question/242581/remapping-the-gazebo-node/
        self.publisher = rospy.Publisher("motor_node/car_cmd",
                                         CarCmd,
                                         queue_size=1)

    def run(self):
        """Main logic loop. Should be called periodically in a loop"""
        running = True
        while running:
            self.screen.blit(self.buttons, (0, 0))

            keys = [False] * 14

            keyboard_input = pygame.key.get_pressed()

            self.check_direction_button(keyboard_input, keys)
            self.check_velocity_button(keyboard_input, keys)

            if keyboard_input[pygame.K_q]:
                rospy.signal_shutdown("Pygame Quit by user input")
                break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Pygame Quit Event")
                    running = False

            pygame.display.flip()
            self.pub_message(keys)
            time.sleep(0.03)

    def check_direction_button(self, keyboard_input, keys):
        """Checks which arrow keys are pressed"""
        if keyboard_input[pygame.K_LEFT]:
            self.screen.blit(self.button_image["left"], (0, 0))
            keys[self.K_LEFT] = True

        if keyboard_input[pygame.K_RIGHT]:
            self.screen.blit(self.button_image["right"], (0, 0))
            keys[self.K_RIGHT] = True

        if keyboard_input[pygame.K_UP]:
            self.screen.blit(self.button_image["forward"], (0, 0))
            keys[self.K_UP] = True

        if keyboard_input[pygame.K_DOWN]:
            self.screen.blit(self.button_image["backward"], (0, 0))
            keys[self.K_DOWN] = True

    def check_velocity_button(self, keyboard_input, keys):
        """Checks which number keys are pressed"""
        if keyboard_input[pygame.K_0]:
            keys[self.K_0] = True

        if keyboard_input[pygame.K_1]:
            keys[self.K_1] = True

        if keyboard_input[pygame.K_2]:
            keys[self.K_2] = True

        if keyboard_input[pygame.K_3]:
            keys[self.K_3] = True

        if keyboard_input[pygame.K_4]:
            keys[self.K_4] = True

        if keyboard_input[pygame.K_5]:
            keys[self.K_5] = True

        if keyboard_input[pygame.K_6]:
            keys[self.K_6] = True

        if keyboard_input[pygame.K_7]:
            keys[self.K_7] = True

        if keyboard_input[pygame.K_8]:
            keys[self.K_8] = True

        if keyboard_input[pygame.K_9]:
            keys[self.K_9] = True

    def pub_message(self, keys):
        """Publishes a CarCmd message to control the car.

        Args:
            keys(list): List of bools corresponding to pressed/unpressed keys
        """
        car_cmd = CarCmd()
        car_cmd.velocity = 0.0
        car_cmd.angle = 0.0

        self.set_velocity_from_keys(keys)

        if keys[self.K_UP]:
            car_cmd.velocity = self.velocity
        if keys[self.K_DOWN]:
            car_cmd.velocity = -self.velocity
        if keys[self.K_LEFT]:
            car_cmd.angle = 100.0 # angle will be clamped by the car itself
        if keys[self.K_RIGHT]:
            car_cmd.angle = -100.0 # angle will be clamped by the car itself

        self.publisher.publish(car_cmd)

    def set_velocity_from_keys(self, keys):
        """Sets the member 'velocity' according to arrow keys pressed

        Args:
            keys (list): List of bools corresponding to pressed/unpressed keys
        """
        if keys[self.K_0]:
            self.velocity = 1.0
        if keys[self.K_1]:
            self.velocity = 0.1
        if keys[self.K_2]:
            self.velocity = 0.2
        if keys[self.K_3]:
            self.velocity = 0.3
        if keys[self.K_4]:
            self.velocity = 0.4
        if keys[self.K_5]:
            self.velocity = 0.5
        if keys[self.K_6]:
            self.velocity = 0.6
        if keys[self.K_7]:
            self.velocity = 0.7
        if keys[self.K_8]:
            self.velocity = 0.8
        if keys[self.K_9]:
            self.velocity = 0.9


def main():
    """Inits the ROS node and KeyboardController."""
    rospy.init_node("keyboard_control_node")

    node = KeyboardController()
    try:
        node.run()
    except rospy.ROSInitException as error:
        rospy.signal_shutdown("{}".format(error))


if __name__ == "__main__":
    main()
