#!/usr/bin/env python3.8

import rospy
import sys

from htrc_controllers.kinova_hand_2_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.kinova_test_system_controller import Controller


def run(pose, angle):
    rospy.init_node("hand_controller", anonymous=True)

    hand = HandController(nb_fingers=2)
    cam = WebcamController(source="/dev/video0")

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    controller.run_hand(pose, angle)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Please provide the pose and angle as arguments in the form:")
        print("rosrun robot_control kinova_test_hand_2.py <robot type> <pose> <angle>")
    else:
        run(sys.argv[2], sys.argv[3])
