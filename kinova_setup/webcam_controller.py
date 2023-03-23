#!/usr/bin/env python3.8

import mediapipe as mp
import cv2
import numpy as np


HAND_LANDMARKS = mp.solutions.hands.HandLandmark


class WebcamController:
    def __init__(self, source: str = "/dev/video0") -> None:
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils

        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

        self.cap_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.cap_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def read_capture(self, hands):
        """
        Process the raw image capture from the webcam to work with MediaPipe Hands.
        """

        # Read the current frame from the webcam
        ret, frame = self.cap.read()

        # Convert frame from BGR to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Flip on the horizontal axis for hand tracking and display
        image = cv2.flip(image, 1)

        # Set flag
        image.flags.writeable = False

        # Get the hand tracking results for this frame
        results = hands.process(image)

        # Set the image to be writable so can display hand tracking data
        image.flags.writeable = True

        # Convert frame from back to BGR from RGB
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        return image, results

    def draw_landmark_results(self, results, image):
        """
        Draw the MediaPipe Hands landmarks and landmark connections to the image.
        """

        for num, hand in enumerate(results.multi_hand_landmarks):
            self.mp_drawing.draw_landmarks(
                image,
                hand,
                self.mp_hands.HAND_CONNECTIONS,
            )

    def get_landmark_data(self, results, required_landmarks):
        """
        Create a dictionary of dictionaries of each landmark coordinate by axis.
        """

        landmark_data = {}
        for hand in results.multi_hand_landmarks:
            landmark_data["x"] = {
                landmark: hand.landmark[landmark].x for landmark in required_landmarks
            }

            landmark_data["y"] = {
                landmark: hand.landmark[landmark].y for landmark in required_landmarks
            }

            landmark_data["z"] = {
                landmark: hand.landmark[landmark].z for landmark in required_landmarks
            }

        return landmark_data

    def display_angle(self, image, landmark_data, angle_dict):
        """
        Display the joint angle data on the image.
        """

        for joint, angle in angle_dict.items():
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
