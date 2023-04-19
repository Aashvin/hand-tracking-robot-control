#!/usr/bin/env python3.8

import rospy

from htrc_controllers.kinova_hand_3_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.system_controller import Controller


def run() -> None:
    """
    Initialise each controller and run the program.
    """

    # Start the hand controller ROS node
    rospy.init_node("hand_controller", anonymous=True)

    # Specify controllers
    hand = HandController(nb_fingers=3)
    cam = WebcamController(source="/dev/video0")

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    # Run one hand
    controller.run_hand()


if __name__ == "__main__":
    run()
