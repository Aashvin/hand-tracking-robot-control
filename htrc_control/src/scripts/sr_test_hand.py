#!/usr/bin/env python3.8

import rospy
import sys

from htrc_controllers.sr_hand_5_finger_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.test_system_controller import Controller


def run(pose, angle):
    rospy.init_node("shadow_robot_hand_controller", anonymous=True)

    hand = HandController(nb_fingers=5, name="right_hand")
    cam = WebcamController(source="/dev/video0")

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    controller.run_hand(pose, angle)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Please provide the pose and angle as arguments in the form:")
        print("rosrun robot_control test_hand.py <pose> <angle>")
    else:
        run(sys.argv[1], sys.argv[2])