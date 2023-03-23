#!/usr/bin/env python3.8

"""
The controller for the shadow robot dexterous hand.
"""

from sr_robot_commander.sr_hand_commander import SrHandCommander

from robot_controller import RobotController
from webcam_controller import HAND_LANDMARKS


class HandController(RobotController):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.nb_fingers: int = 5
        self.controller: SrHandCommander = SrHandCommander(name=name)

    def set_required_landmarks(self) -> None:
        self.required_landmarks = [
            HAND_LANDMARKS.INDEX_FINGER_MCP,
            HAND_LANDMARKS.INDEX_FINGER_PIP,
            HAND_LANDMARKS.INDEX_FINGER_DIP,
            HAND_LANDMARKS.MIDDLE_FINGER_MCP,
            HAND_LANDMARKS.MIDDLE_FINGER_PIP,
            HAND_LANDMARKS.MIDDLE_FINGER_DIP,
            HAND_LANDMARKS.RING_FINGER_MCP,
            HAND_LANDMARKS.RING_FINGER_PIP,
            HAND_LANDMARKS.RING_FINGER_DIP,
            HAND_LANDMARKS.PINKY_MCP,
            HAND_LANDMARKS.PINKY_PIP,
            HAND_LANDMARKS.PINKY_DIP,
            HAND_LANDMARKS.THUMB_MCP,
            HAND_LANDMARKS.THUMB_IP,
            HAND_LANDMARKS.THUMB_TIP,
            HAND_LANDMARKS.WRIST,
        ]

    def move_to_start_pose(self) -> None:
        self.controller.move_to_named_target("open")

    def publish_move(self) -> None:
        # Target of thread so return from while loop once ready to kill thread
        while True:
            # Get hand controller angles from the queue
            angle_dict = self.data_queue.get()

            # End if the user has quit the main program
            if angle_dict == "END":
                return

            # If the queue contains something, process and send it to the hand
            if angle_dict is not None:
                flex = {
                    "rh_FFJ2": angle_dict[HAND_LANDMARKS.INDEX_FINGER_PIP],
                    "rh_FFJ3": angle_dict[HAND_LANDMARKS.INDEX_FINGER_MCP],
                    "rh_MFJ2": angle_dict[HAND_LANDMARKS.MIDDLE_FINGER_PIP],
                    "rh_MFJ3": angle_dict[HAND_LANDMARKS.MIDDLE_FINGER_MCP],
                    "rh_RFJ2": angle_dict[HAND_LANDMARKS.RING_FINGER_PIP],
                    "rh_RFJ3": angle_dict[HAND_LANDMARKS.RING_FINGER_MCP],
                    "rh_LFJ2": angle_dict[HAND_LANDMARKS.PINKY_PIP],
                    "rh_LFJ3": angle_dict[HAND_LANDMARKS.PINKY_MCP],
                    "rh_THJ2": angle_dict[HAND_LANDMARKS.THUMB_IP],
                    "rh_THJ3": angle_dict[HAND_LANDMARKS.THUMB_MCP],
                }

                self.controller.move_to_joint_value_target_unsafe(
                    flex, wait=False, angle_degrees=True
                )
