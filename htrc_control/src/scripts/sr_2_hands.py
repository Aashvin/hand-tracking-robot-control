#!/usr/bin/env python3.8

import rospy

from htrc_controllers.sr_hand_5_finger_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.system_controller import Controller


def run() -> None:
    """
    Initialise each controller and run the program.
    """

    # Start the hand controller ROS node
    rospy.init_node("shadow_robot_hand_controller", anonymous=True)

    # Specify controllers
    right_hand = HandController(nb_fingers=5, name="right_hand")
    left_hand = HandController(nb_fingers=5, name="left_hand")
    cam = WebcamController(source="/dev/video0", num_hands=2)

    controller = Controller(
        webcam_controller=cam, hand_controller=right_hand, hand2_controller=left_hand
    )

    # Run two hands
    controller.run_2_hands()


if __name__ == "__main__":
    run()
