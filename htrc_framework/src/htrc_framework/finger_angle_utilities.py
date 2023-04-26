#!/usr/bin/env python3.8

import numpy as np
from typing import Dict, List

from htrc_framework.webcam_controller import HAND_LANDMARKS


def calculate_angles(joints: Dict[str, List[float]]):
    """
    Calculates the angle of the middle joint of 3 consecutive joints.
    Models 2 consecutive joints as vectors to calculate the angle between them.
    """

    # Calculate the two vectors
    # From upper joint to middle joint and from lower joint to middle joint
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

    # Calculate the magnitudes of those vectors and normalise them
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

    # Calculate the actual angle of the middle joint of the 3 arguments
    angle = 180 - np.degrees(np.arccos(dot))

    return angle


def finger_angles(
    nb_fingers: int,
    required_landmarks: List[HAND_LANDMARKS],
    landmark_data: Dict[str, Dict[HAND_LANDMARKS, float]],
) -> Dict[HAND_LANDMARKS, float]:
    """
    Gets the landmark data of 3 consecutive joints to calculate the angle of the middle one.
    Does this for each finger's upper (PIP) joint and low (MCP) joint.
    required_landmarks must be in a proper format for this function to work correctly.
    """

    # Dictionary that will contain the PIP and MCP angle joints
    angle_dict = {}

    # Loop through each finger indicated
    for finger in range(nb_fingers):
        # Construct dictionary of x, y, z coordinates of top 3 joints
        upper_joints = {
            axis: [
                landmark_data[axis][required_landmarks[finger * 3 + i]]
                for i in range(3)
            ]
            for axis in ["x", "y", "z"]
        }

        # Construct dictionary of x, y, z coordinates of bottom 2 joints and wrist
        lower_joints = {
            axis: [
                landmark_data[axis][required_landmarks[finger * 3 + i]]
                if i != -1
                else landmark_data[axis][required_landmarks[-1]]
                for i in range(-1, 2)
            ]
            for axis in ["x", "y", "z"]
        }

        # Get the MCP and PIP joint names of the current finger
        lower_joint = required_landmarks[finger * 3]
        upper_joint = required_landmarks[finger * 3 + 1]

        # Add the calculated angles to the angle_dict
        # Clip the MCP joint angle to be non-negative
        angle_dict[lower_joint] = np.clip(
            (calculate_angles(lower_joints) - 45) * 2 + 45, 0, 90
        )
        angle_dict[upper_joint] = calculate_angles(upper_joints)

    return angle_dict
