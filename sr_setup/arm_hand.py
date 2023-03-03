#!/usr/bin/env python3.8

import rospy

from arm_controller import ArmController
from hand_controller import HandController
from webcam_controller import WebcamController
from system_controller import Controller


def run():
    rospy.init_node("arm_hand_controller", anonymous=True)

    hand = HandController(name="right_hand")
    arm = ArmController(name="right_arm")
    cam = WebcamController()

    controller = Controller(
        webcam_controller=cam, hand_controller=hand, arm_controller=arm
    )

    controller.run_arm_hand()


if __name__ == "__main__":
    run()
