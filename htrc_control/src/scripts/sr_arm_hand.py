#!/usr/bin/env python3.8

import rospy

from htrc_controllers.sr_arm_position_controller import ArmController
from htrc_controllers.sr_hand_5_finger_controller import HandController
from htrc_framework.webcam_controller import WebcamController
from htrc_framework.system_controller import Controller


def run() -> None:
    """
    Initialise each controller and run the program.
    """

    # Start the hand controller ROS node
    rospy.init_node("shadow_robot_arm_hand_controller", anonymous=True)

    # Specify controllers
    hand = HandController(nb_fingers=5, name="right_hand")
    arm = ArmController(name="right_arm")
    cam = WebcamController(source="/dev/video0")

    controller = Controller(
        webcam_controller=cam, hand_controller=hand, arm_controller=arm
    )

    # Run arm and hand
    controller.run_arm_hand()


if __name__ == "__main__":
    run()
