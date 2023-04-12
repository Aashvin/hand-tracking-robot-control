#!/usr/bin/env python3.8

import rospy

from htrc_controllers.kinova_hand_3_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.system_controller import Controller


def run():
    rospy.init_node("hand_controller", anonymous=True)

    hand = HandController()
    cam = WebcamController(source="/dev/video0")

    controller = Controller(webcam_controller=cam, hand_controller=hand)

    controller.run_hand()


if __name__ == "__main__":
    run()
