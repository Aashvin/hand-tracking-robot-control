#!/usr/bin/env python3.8

import rospy

from hand_controller import HandController
from webcam_controller import WebcamController
from system_controller import Controller


def run():
    rospy.init_node("shadow_robot_hand_controller", anonymous=True)

    right_hand = HandController(name="right_hand")
    left_hand = HandController(name="left_hand")
    cam = WebcamController(source="/dev/video0", num_hands=2)

    controller = Controller(webcam_controller=cam, hand_controller=right_hand, hand2_controller=left_hand)

    controller.run_2_hands()


if __name__ == "__main__":
    run()
