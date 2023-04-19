#!/usr/bin/env python3.8

import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import numpy as np
import os
import time
from typing import Dict, List, Tuple

from mediapipe.tasks.python.components.containers.landmark import NormalizedLandmark
from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarkerResult

HAND_LANDMARKS = mp.solutions.hands.HandLandmark


class WebcamController:
    def __init__(self, source: str = "/dev/video0", num_hands: int = 1) -> None:
        # Set the input video capture
        self.cap = cv2.VideoCapture(source)

        ## Set the fourcc code of the VideoWriter
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))

        # Set the dimensions of the capture
        self.cap_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.cap_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        # Initialise the hand detector from the hand_landmarker.task file
        tracker_task_path = (
            os.path.realpath(os.path.dirname(__file__)).replace("\\", "/")
            + "/hand_landmarker.task"
        )
        base_options = mp.tasks.BaseOptions(model_asset_path=tracker_task_path)
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=num_hands,
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            min_hand_detection_confidence=0.7,
            min_tracking_confidence=0.7,
        )
        self.detector = mp.tasks.vision.HandLandmarker.create_from_options(options)

    def read_capture(self) -> Tuple[mp.Image, HandLandmarkerResult]:
        """
        Process the raw image capture from the webcam to work with MediaPipe Hands.
        """

        # Read the current frame from the webcam and flip on the horizontal axis for correct orientation
        successful_read = False
        while not successful_read:
            successful_read, frame = self.cap.read()
        frame = cv2.flip(frame, 1)

        # Convert current frame into a MediaPipe Image
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

        # Get results from hand tracking
        results = self.detector.detect_for_video(mp_image, int(time.time() * 1000))

        return mp_image, results

    def draw_landmark_results(
        self, results: List[NormalizedLandmark], image: np.ndarray
    ) -> np.ndarray:
        """
        Draw the MediaPipe Hands landmarks and landmark connections to the image.
        """

        annotated_image = np.copy(image)

        # Construct the hand landmarks in format to draw on image
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend(
            [
                landmark_pb2.NormalizedLandmark(
                    x=landmark.x, y=landmark.y, z=landmark.z
                )
                for landmark in results
            ]
        )

        # Draw the hand landmarks and landmark connections on the image
        mp.solutions.drawing_utils.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            mp.solutions.hands.HAND_CONNECTIONS,
            mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
            mp.solutions.drawing_styles.get_default_hand_connections_style(),
        )

        return annotated_image

    def get_landmark_data(
        self,
        results: List[NormalizedLandmark],
        required_landmarks: List[HAND_LANDMARKS],
    ) -> Dict[str, Dict[HAND_LANDMARKS, float]]:
        """
        Create a dictionary of dictionaries of each landmark coordinate by axis.
        """

        print(results)

        landmark_data = {
            "x": {landmark: results[landmark].x for landmark in required_landmarks},
            "y": {landmark: results[landmark].y for landmark in required_landmarks},
            "z": {landmark: results[landmark].z for landmark in required_landmarks},
        }

        return landmark_data

    def display_angle(
        self,
        image: np.ndarray,
        landmark_data: Dict[str, Dict[HAND_LANDMARKS, float]],
        angle_dict: Dict[HAND_LANDMARKS, float],
    ):
        """
        Display the joint angle data on the image.
        """

        # For all joints and angle pairs in the required PIP and MCP joints
        for joint, angle in angle_dict.items():
            # Get the normalised x and y coordinates of the current landmark
            coordinates = np.array(
                [landmark_data["x"][joint], landmark_data["y"][joint]]
            )

            # Put the angle text on the image next to the landmark it corresponds to
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
