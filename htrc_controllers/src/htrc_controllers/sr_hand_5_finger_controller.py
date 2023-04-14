#!/usr/bin/env python3.8

"""
The controller for the shadow robot dexterous hand.
"""

from sr_robot_commander.sr_hand_commander import SrHandCommander

from htrc_framework.base_robot_controllers import BaseHandController
from htrc_framework.webcam_controller import HAND_LANDMARKS


class HandController(BaseHandController):
    def __init__(self, nb_fingers: int, name: str) -> None:
        super().__init__(nb_fingers)
        self.commander: SrHandCommander = SrHandCommander(name=name)

        if name == "right_hand":
            self.prefix = "rh"
        elif name == "left_hand":
            self.prefix = "lh"
        else:
            "Please use 'name=right_hand' or 'name=left_hand' to set a joint prefix. The hand will not be contraollable otherwise."

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
        self.commander.move_to_named_target("open")

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
                    f"{self.prefix}_FFJ2": angle_dict[HAND_LANDMARKS.INDEX_FINGER_PIP],
                    f"{self.prefix}_FFJ3": angle_dict[HAND_LANDMARKS.INDEX_FINGER_MCP],
                    f"{self.prefix}_MFJ2": angle_dict[HAND_LANDMARKS.MIDDLE_FINGER_PIP],
                    f"{self.prefix}_MFJ3": angle_dict[HAND_LANDMARKS.MIDDLE_FINGER_MCP],
                    f"{self.prefix}_RFJ2": angle_dict[HAND_LANDMARKS.RING_FINGER_PIP],
                    f"{self.prefix}_RFJ3": angle_dict[HAND_LANDMARKS.RING_FINGER_MCP],
                    f"{self.prefix}_LFJ2": angle_dict[HAND_LANDMARKS.PINKY_PIP],
                    f"{self.prefix}_LFJ3": angle_dict[HAND_LANDMARKS.PINKY_MCP],
                    f"{self.prefix}_THJ1": angle_dict[HAND_LANDMARKS.THUMB_IP],
                    f"{self.prefix}_THJ2": angle_dict[HAND_LANDMARKS.THUMB_MCP],
                }

                self.commander.move_to_joint_value_target_unsafe(
                    flex, wait=False, angle_degrees=True
                )
