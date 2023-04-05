#!/usr/bin/env python3.8

import rospy
import sys

from hand_controller import HandController
from webcam_controller import WebcamController
from test_system_controller import Controller


def run(pose, angle):
    rospy.init_node("shadow_robot_hand_controller", anonymous=True)

    hand = HandController(name="right_hand")
    cam = WebcamController(source="/dev/video0")

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    controller.run_hand(pose, angle)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Please provide the angle and pose as arguments in the form:")
        print("rosrun robot_control test_hand.py <angle> <pose>")
    else:
        run(sys.argv[1], sys.argv[2])
