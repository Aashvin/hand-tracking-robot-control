#!/usr/bin/env python3.8

import numpy as np


def calculate_angles(joints):
    """
    Calculates the angle of the middle joint of 3 consecutive joints.
    Models 2 consecutive joints as vectors to calculate the angle between them.
    """

    # Calculate the two vectors
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

    return angle


def finger_angles(nb_fingers, required_landmarks, landmark_data):
    """
    Gets the landmark data of 3 consecutive joints to calculate the angle of the middle one.
    Does this for each finger's upper (PIP) joint and low (MCP) joint.
    required_landmarks must be in a proper format for this function to work correctly.
    """

    angle_dict = {}
    for finger in range(nb_fingers):
        upper_joints = {
            dim: [
                landmark_data[dim][required_landmarks[finger * 3 + i]] for i in range(3)
            ]
            for dim in ["x", "y", "z"]
        }
        lower_joints = {
            dim: [
                landmark_data[dim][required_landmarks[finger * 3 + i]]
                if i != -1
                else landmark_data[dim][required_landmarks[-1]]
                for i in range(-1, 2)
            ]
            for dim in ["x", "y", "z"]
        }

        lower_joint = required_landmarks[finger * 3]
        upper_joint = required_landmarks[finger * 3 + 1]

        angle_dict[lower_joint] = np.clip(
            (calculate_angles(lower_joints) - 45) * 2 + 45, 0, 90
        )
        angle_dict[upper_joint] = calculate_angles(upper_joints)

    return angle_dict
