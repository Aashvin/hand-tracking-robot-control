#!/usr/bin/env python3.8

import numpy as np
from sr_robot_commander.sr_hand_commander import SrHandCommander

from robot_controller import RobotController
from webcam_controller import HAND_LANDMARKS


class HandController(RobotController):
    def __init__(self, name: str) -> None:
        super().__init__()
        self.controller = SrHandCommander(name=name)

    def set_required_landmarks(self):
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

    def move_to_start_pose(self):
        self.controller.move_to_named_target("open")

    def calculate_angles(self, joints):
        high_mid_vec = [
            joints["x"][2] - joints["x"][1],
            joints["y"][2] - joints["y"][1],
            joints["z"][2] - joints["z"][1],
        ]

        low_mid_vec = [
            joints["x"][0] - joints["x"][1],
            joints["y"][0] - joints["y"][1],
            joints["z"][0] - joints["z"][1],
        ]

        # Calculate the magnitudes of those vectors to normalise them
        high_mid_vec_mag = np.sqrt(
            high_mid_vec[0] ** 2 + high_mid_vec[1] ** 2 + high_mid_vec[2] ** 2
        )
        high_mid_vec_normalised = [
            high_mid_vec[0] / high_mid_vec_mag,
            high_mid_vec[1] / high_mid_vec_mag,
            high_mid_vec[2] / high_mid_vec_mag,
        ]

        low_mid_vec_mag = np.sqrt(
            low_mid_vec[0] ** 2 + low_mid_vec[1] ** 2 + low_mid_vec[2] ** 2
        )
        low_mid_vec_normalised = [
            low_mid_vec[0] / low_mid_vec_mag,
            low_mid_vec[1] / low_mid_vec_mag,
            low_mid_vec[2] / low_mid_vec_mag,
        ]

        # Get the dot product between the two vectors
        # Equivalent to the cosine of the angle between the two vectors
        dot = (
            high_mid_vec_normalised[0] * low_mid_vec_normalised[0]
            + high_mid_vec_normalised[1] * low_mid_vec_normalised[1]
            + high_mid_vec_normalised[2] * low_mid_vec_normalised[2]
        )

        # Calculate the actual angle
        angle = 180 - np.degrees(np.arccos(dot))

        # If the angle is calculated for a lower joint, clip it between 0 and 90
        # Avoids out of bounds angles being published to the robot

        return angle

    def finger_angles(self, landmark_data):
        angle_dict = {}
        for finger in range(5):
            upper_joints = {
                dim: [
                    landmark_data[dim][self.required_landmarks[finger * 3 + i]]
                    for i in range(3)
                ]
                for dim in ["x", "y", "z"]
            }
            lower_joints = {
                dim: [
                    landmark_data[dim][self.required_landmarks[finger * 3 + i]]
                    if i != -1
                    else landmark_data[dim][self.required_landmarks[-1]]
                    for i in range(-1, 2)
                ]
                for dim in ["x", "y", "z"]
            }

            lower_joint = self.required_landmarks[finger * 3]
            upper_joint = self.required_landmarks[finger * 3 + 1]

            angle_dict[lower_joint] = np.clip(
                (self.calculate_angles(lower_joints) - 45) * 2 + 45, 0, 90
            )
            angle_dict[upper_joint] = self.calculate_angles(upper_joints)

        return angle_dict

    def publish_move(self):
        while True:
            angle_dict = self.data_queue.get()

            if angle_dict == "END":
                return

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
