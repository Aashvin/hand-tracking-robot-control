#!/usr/bin/env python3.8

import cv2
import numpy as np
import mediapipe as mp


class DisplayController:
    def __init__(self) -> None:
        self.mp_drawing = mp.solutions.drawing_utils

    def display_angle(self, image, landmark_data, angle_dict):
        for joint, angle in angle_dict.items:
            coordinates = np.array(
                [landmark_data["x"][joint], landmark_data["y"][joint]]
            )

            cv2.putText(
                image,
                str(round(angle, 2)),
                tuple(
                    np.multiply(coordinates, [self.cap_width, self.cap_height]).astype(
                        int
                    )
                ),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
