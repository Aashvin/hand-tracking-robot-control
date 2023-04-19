#!/usr/bin/env python3.8

import rospy
import sys

from htrc_controllers.kinova_hand_3_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.kinova_test_system_controller import Controller


def run(pose: int, angle: int) -> None:
    """
    Initialise each controller and run the program.
    """

    # Start the hand controller ROS node
    rospy.init_node("hand_controller", anonymous=True)

    # Specify controllers
    hand = HandController(nb_fingers=3)
    cam = WebcamController(source="/dev/video0")

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    # Run one hand test at the desired pose and angle
    controller.run_hand(pose, angle)


if __name__ == "__main__":
    # Make sure the right arguments are provided
    if len(sys.argv) < 3:
        print("Please provide the pose and angle as arguments in the form:")
        print("rosrun robot_control kinova_test_hand_3.py <robot type> <pose> <angle>")
    else:
        run(sys.argv[2], sys.argv[3])
