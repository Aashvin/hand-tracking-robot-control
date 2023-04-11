#!/usr/bin/env python3.8

import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import cv2
import numpy as np
import os
import time


HAND_LANDMARKS = mp.solutions.hands.HandLandmark


class WebcamController:
    def __init__(self, source: str = "/dev/video0", num_hands: int = 1) -> None:
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

        self.cap_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.cap_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        tracker_task_path = os.path.realpath(os.path.dirname(__file__)).replace("\\", "/") + "/hand_landmarker.task"
        self.base_options = mp.tasks.BaseOptions(model_asset_path=tracker_task_path)
        self.options = mp.tasks.vision.HandLandmarkerOptions(base_options=self.base_options, num_hands=num_hands, running_mode=mp.tasks.vision.RunningMode.VIDEO, min_hand_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.detector = mp.tasks.vision.HandLandmarker.create_from_options(self.options)

    def read_capture(self):
        """
        Process the raw image capture from the webcam to work with MediaPipe Hands.
        """

        # Read the current frame from the webcam and flip on the horizontal axis for correct orientation
        frame = cv2.flip(self.cap.read()[1], 1)

        # Convert current frame into a MediaPipe Image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

        results = self.detector.detect_for_video(mp_image, int(time.time() * 1000))

        return mp_image, results

    def draw_landmark_results(self, results, image):
        """
        Draw the MediaPipe Hands landmarks and landmark connections to the image.
        """

        annotated_image = np.copy(image)

        # Draw the hand landmarks.
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend(
            [
                landmark_pb2.NormalizedLandmark(
                    x=landmark.x, y=landmark.y, z=landmark.z
                )
                for landmark in results
            ]
        )
        mp.solutions.drawing_utils.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            mp.solutions.hands.HAND_CONNECTIONS,
            mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
            mp.solutions.drawing_styles.get_default_hand_connections_style(),
        )

        return annotated_image

    def get_landmark_data(self, results, required_landmarks):
        """
        Create a dictionary of dictionaries of each landmark coordinate by axis.
        """

        landmark_data = {
            "x": {landmark: results[landmark].x for landmark in required_landmarks},
            "y": {landmark: results[landmark].y for landmark in required_landmarks},
            "z": {landmark: results[landmark].z for landmark in required_landmarks},
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
