#!/usr/bin/env python3.8

import rospy
import sys

from htrc_controllers.sr_arm_position_controller import ArmController
from htrc_controllers.sr_hand_5_finger_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.sr_test_system_controller import Controller


def run(refresh_rate):
    rospy.init_node("shadow_robot_arm_hand_controller", anonymous=True)

    hand = HandController(nb_fingers=5, name="right_hand")
    arm = ArmController(name="right_arm")
    cam = WebcamController(source="/dev/video0")

    controller = Controller(
        webcam_controller=cam, hand_controller=hand, arm_controller=arm
    )

    controller.run_arm_hand(refresh_rate)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide the refresh rate as an argument in the form:")
        print("rosrun robot_control sr_test_arm_hand.py <refresh rate>")
    else:
        run(float(sys.argv[1]))
