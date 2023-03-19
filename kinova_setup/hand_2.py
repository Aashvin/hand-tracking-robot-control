#!/usr/bin/env python3.8

import rospy

from hand_2_controller import HandController
from webcam_controller import WebcamController
from system_controller import Controller


def run():
    rospy.init_node("hand_controller", anonymous=True)

    hand = HandController()
    cam = WebcamController()

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    controller.run_hand()


if __name__ == "__main__":
    run()
